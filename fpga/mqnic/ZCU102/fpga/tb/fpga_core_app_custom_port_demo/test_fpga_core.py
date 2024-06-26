# SPDX-License-Identifier: BSD-2-Clause-Views
# Copyright (c) 2020-2023 The Regents of the University of California

import logging
import os
import struct
import sys

import scapy.utils
from scapy.layers.l2 import Ether
from scapy.layers.inet import IP, UDP

import cocotb_test.simulator

import cocotb
from cocotb.log import SimLog
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

from cocotbext.axi import AddressSpace
from cocotbext.axi import AxiLiteMaster, AxiLiteBus
from cocotbext.axi import AxiSlave, AxiBus
from cocotbext.eth import XgmiiSource, XgmiiSink, XgmiiFrame

try:
    import mqnic
except ImportError:
    # attempt import from current directory
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    try:
        import mqnic
    finally:
        del sys.path[0]


class TB(object):
    def __init__(self, dut):
        self.dut = dut

        self.log = SimLog("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        cocotb.start_soon(Clock(dut.clk_300mhz, 3332, units="ps").start())

        # AXI
        self.address_space = AddressSpace()
        self.pool = self.address_space.create_pool(0, 0x8000_0000)

        self.axil_master = AxiLiteMaster(AxiLiteBus.from_prefix(dut, "s_axil_ctrl"), dut.clk_300mhz, dut.rst_300mhz)
        self.address_space.register_region(self.axil_master, 0x10_0000_0000)
        self.hw_regs = self.address_space.create_window(0x10_0000_0000, self.axil_master.size)

        self.axi_slave = AxiSlave(AxiBus.from_prefix(dut, "m_axi"), dut.clk_300mhz, dut.rst_300mhz, self.address_space)

        self.driver = mqnic.Driver()

        cocotb.start_soon(Clock(dut.ptp_clk, 6.4, units="ns").start())
        dut.ptp_rst.setimmediatevalue(0)
        cocotb.start_soon(Clock(dut.ptp_sample_clk, 8, units="ns").start())

        # Ethernet
        self.sfp_source = []
        self.sfp_sink = []

        for k in range(4):
            cocotb.start_soon(Clock(getattr(dut, f"sfp{k}_rx_clk"), 6.4, units="ns").start())
            source = XgmiiSource(getattr(dut, f"sfp{k}_rxd"), getattr(dut, f"sfp{k}_rxc"), getattr(dut, f"sfp{k}_rx_clk"), getattr(dut, f"sfp{k}_rx_rst"))
            self.sfp_source.append(source)
            cocotb.start_soon(Clock(getattr(dut, f"sfp{k}_tx_clk"), 6.4, units="ns").start())
            sink = XgmiiSink(getattr(dut, f"sfp{k}_txd"), getattr(dut, f"sfp{k}_txc"), getattr(dut, f"sfp{k}_tx_clk"), getattr(dut, f"sfp{k}_tx_rst"))
            self.sfp_sink.append(sink)
            getattr(dut, f"sfp{k}_rx_status").setimmediatevalue(1)
            getattr(dut, f"sfp{k}_rx_error_count").setimmediatevalue(0)

        cocotb.start_soon(Clock(dut.sfp_drp_clk, 8, units="ns").start())
        dut.sfp_drp_rst.setimmediatevalue(0)
        dut.sfp_drp_do.setimmediatevalue(0)
        dut.sfp_drp_rdy.setimmediatevalue(0)

        dut.btnu.setimmediatevalue(0)
        dut.btnl.setimmediatevalue(0)
        dut.btnd.setimmediatevalue(0)
        dut.btnr.setimmediatevalue(0)
        dut.btnc.setimmediatevalue(0)
        dut.sw.setimmediatevalue(0)

        self.loopback_enable = False
        cocotb.start_soon(self._run_loopback())

    async def init(self):

        self.dut.rst_300mhz.setimmediatevalue(0)
        self.dut.ptp_rst.setimmediatevalue(0)
        for k in range(4):
            getattr(self.dut, f"sfp{k}_rx_rst").setimmediatevalue(0)
            getattr(self.dut, f"sfp{k}_tx_rst").setimmediatevalue(0)

        await RisingEdge(self.dut.clk_300mhz)
        await RisingEdge(self.dut.clk_300mhz)

        self.dut.rst_300mhz.value = 1
        self.dut.ptp_rst.setimmediatevalue(1)
        for k in range(4):
            getattr(self.dut, f"sfp{k}_rx_rst").setimmediatevalue(1)
            getattr(self.dut, f"sfp{k}_tx_rst").setimmediatevalue(1)

        await RisingEdge(self.dut.clk_300mhz)
        await RisingEdge(self.dut.clk_300mhz)

        self.dut.rst_300mhz.value = 0
        self.dut.ptp_rst.setimmediatevalue(0)
        for k in range(4):
            getattr(self.dut, f"sfp{k}_rx_rst").setimmediatevalue(0)
            getattr(self.dut, f"sfp{k}_tx_rst").setimmediatevalue(0)

    async def _run_loopback(self):
        while True:
            await RisingEdge(self.dut.clk_300mhz)

            if self.loopback_enable:
                for x in range(len(self.sfp_sink)):
                    if not self.sfp_sink[x].empty():
                        await self.sfp_source[x].send(await self.sfp_sink[x].recv())


@cocotb.test()
async def run_test_nic(dut):

    tb = TB(dut)

    await tb.init()

    tb.log.info("Init driver")
    await tb.driver.init_axi_dev(tb.pool, tb.hw_regs, irq=dut.irq)
    for interface in tb.driver.interfaces:
        await interface.ndevs[0].open()

    # wait for all writes to complete
    await tb.driver.hw_regs.read_dword(0)
    tb.log.info("Init complete")

    tb.log.info("Send and receive single packet")

    data = bytearray([x % 256 for x in range(1024)])

    await tb.driver.interfaces[0].ndevs[0].start_xmit(data, 0)

    pkt = await tb.sfp_sink[0].recv()
    tb.log.info("Packet: %s", pkt)

    await tb.sfp_source[0].send(pkt)

    pkt = await tb.driver.interfaces[0].ndevs[0].recv()

    tb.log.info("Packet: %s", pkt)
    assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

    # await tb.driver.interfaces[1].ndevs[0].start_xmit(data, 0)

    # pkt = await tb.sfp_sink[1].recv()
    # tb.log.info("Packet: %s", pkt)

    # await tb.sfp_source[1].send(pkt)

    # pkt = await tb.driver.interfaces[1].ndevs[0].recv()

    # tb.log.info("Packet: %s", pkt)
    # assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

    tb.log.info("RX and TX checksum tests")

    payload = bytes([x % 256 for x in range(256)])
    eth = Ether(src='5A:51:52:53:54:55', dst='DA:D1:D2:D3:D4:D5')
    ip = IP(src='192.168.1.100', dst='192.168.1.101')
    udp = UDP(sport=1, dport=2)
    test_pkt = eth / ip / udp / payload

    test_pkt2 = test_pkt.copy()
    test_pkt2[UDP].chksum = scapy.utils.checksum(bytes(test_pkt2[UDP]))

    await tb.driver.interfaces[0].ndevs[0].start_xmit(test_pkt2.build(), 0, 34, 6)

    pkt = await tb.sfp_sink[0].recv()
    tb.log.info("Packet: %s", pkt)

    await tb.sfp_source[0].send(pkt)

    pkt = await tb.driver.interfaces[0].ndevs[0].recv()

    tb.log.info("Packet: %s", pkt)
    assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff
    assert Ether(pkt.data).build() == test_pkt.build()

    tb.log.info("Queue mapping offset test")

    data = bytearray([x % 256 for x in range(1024)])

    tb.loopback_enable = True

    for k in range(4):
        await tb.driver.interfaces[0].set_rx_queue_map_indir_table(0, 0, tb.driver.interfaces[0].ndevs[0].rxq[k].index)

        await tb.driver.interfaces[0].ndevs[0].start_xmit(data, 0)

        pkt = await tb.driver.interfaces[0].ndevs[0].recv()

        tb.log.info("Packet: %s", pkt)
        if tb.driver.interfaces[0].if_feature_rx_csum:
            assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff
        assert pkt.queue == tb.driver.interfaces[0].ndevs[0].rxq[k].index

    tb.loopback_enable = False

    await tb.driver.interfaces[0].ndevs[0].update_rx_queue_map_indir_table()

    tb.log.info("Queue mapping RSS mask test")

    await tb.driver.interfaces[0].set_rx_queue_map_rss_mask(0, 0x00000003)

    tb.loopback_enable = True

    queues = set()

    for k in range(64):
        payload = bytes([x % 256 for x in range(256)])
        eth = Ether(src='5A:51:52:53:54:55', dst='DA:D1:D2:D3:D4:D5')
        ip = IP(src='192.168.1.100', dst='192.168.1.101')
        udp = UDP(sport=1, dport=k+0)
        test_pkt = eth / ip / udp / payload

        if tb.driver.interfaces[0].if_feature_tx_csum:
            test_pkt2 = test_pkt.copy()
            test_pkt2[UDP].chksum = scapy.utils.checksum(bytes(test_pkt2[UDP]))

            await tb.driver.interfaces[0].ndevs[0].start_xmit(test_pkt2.build(), 0, 34, 6)
        else:
            await tb.driver.interfaces[0].ndevs[0].start_xmit(test_pkt.build(), 0)

    for k in range(64):
        pkt = await tb.driver.interfaces[0].ndevs[0].recv()

        tb.log.info("Packet: %s", pkt)
        if tb.driver.interfaces[0].if_feature_rx_csum:
            assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

        queues.add(pkt.queue)

    assert len(queues) == 4

    tb.loopback_enable = False

    await tb.driver.interfaces[0].set_rx_queue_map_rss_mask(0, 0xffffffff)

    tb.log.info("Multiple small packets")

    count = 64

    pkts = [bytearray([(x+k) % 256 for x in range(60)]) for k in range(count)]

    tb.loopback_enable = True

    for p in pkts:
        await tb.driver.interfaces[0].ndevs[0].start_xmit(p, 0)

    for k in range(count):
        pkt = await tb.driver.interfaces[0].ndevs[0].recv()

        tb.log.info("Packet: %s", pkt)
        assert pkt.data == pkts[k]
        if tb.driver.interfaces[0].if_feature_rx_csum:
            assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

    tb.loopback_enable = False

    tb.log.info("Multiple TX queues")

    count = 1024

    pkts = [bytearray([(x+k) % 256 for x in range(60)]) for k in range(count)]

    tb.loopback_enable = True

    for k in range(len(pkts)):
        await tb.driver.interfaces[0].ndevs[0].start_xmit(pkts[k], k % tb.driver.interfaces[0].ndevs[0].txq_count)

    for k in range(count):
        pkt = await tb.driver.interfaces[0].ndevs[0].recv()

        tb.log.info("Packet: %s", pkt)
        if tb.driver.interfaces[0].if_feature_rx_csum:
            assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

    tb.loopback_enable = False

    tb.log.info("Multiple large packets")

    count = 64

    pkts = [bytearray([(x+k) % 256 for x in range(1514)]) for k in range(count)]

    tb.loopback_enable = True

    for p in pkts:
        await tb.driver.interfaces[0].ndevs[0].start_xmit(p, 0)

    for k in range(count):
        pkt = await tb.driver.interfaces[0].ndevs[0].recv()

        tb.log.info("Packet: %s", pkt)
        assert pkt.data == pkts[k]
        if tb.driver.interfaces[0].if_feature_rx_csum:
            assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

    tb.loopback_enable = False

    tb.log.info("Jumbo frames")

    count = 64

    pkts = [bytearray([(x+k) % 256 for x in range(9014)]) for k in range(count)]

    tb.loopback_enable = True

    for p in pkts:
        await tb.driver.interfaces[0].ndevs[0].start_xmit(p, 0)

    for k in range(count):
        pkt = await tb.driver.interfaces[0].ndevs[0].recv()

        tb.log.info("Packet: %s", pkt)
        assert pkt.data == pkts[k]
        if tb.driver.interfaces[0].if_feature_rx_csum:
            assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

    tb.loopback_enable = False

    if len(tb.driver.interfaces) > 1:
        tb.log.info("All interfaces")

        count = 64

        pkts = [bytearray([(x+k) % 256 for x in range(1514)]) for k in range(count)]

        tb.loopback_enable = True

        for k, p in enumerate(pkts):
            await tb.driver.interfaces[k % len(tb.driver.interfaces)].ndevs[0].start_xmit(p, 0)

        for k in range(count):
            pkt = await tb.driver.interfaces[k % len(tb.driver.interfaces)].ndevs[0].recv()

            tb.log.info("Packet: %s", pkt)
            assert pkt.data == pkts[k]
            if tb.driver.interfaces[0].if_feature_rx_csum:
                assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

        tb.loopback_enable = False

    if len(tb.driver.interfaces[0].ndevs) > 1:
        tb.log.info("All interface 0 netdevs")

        for ndev in tb.driver.interfaces[0].ndevs:
            if not ndev.port_up:
                await ndev.open()

        count = 64

        pkts = [bytearray([(x+k) % 256 for x in range(1514)]) for k in range(count)]

        tb.loopback_enable = True

        queues = set()

        for k, p in enumerate(pkts):
            await tb.driver.interfaces[0].ndevs[k % len(tb.driver.interfaces[0].ndevs)].start_xmit(p, 0)

        for k in range(count):
            pkt = await tb.driver.interfaces[0].ndevs[k % len(tb.driver.interfaces[0].ndevs)].recv()

            tb.log.info("Packet: %s", pkt)
            assert pkt.data == pkts[k]
            if tb.driver.interfaces[0].if_feature_rx_csum:
                assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

        tb.loopback_enable = False

    if tb.driver.interfaces[0].if_feature_lfc:
        tb.log.info("Test LFC pause frame RX")

        await tb.driver.interfaces[0].ports[0].set_lfc_ctrl(mqnic.MQNIC_PORT_LFC_CTRL_TX_LFC_EN | mqnic.MQNIC_PORT_LFC_CTRL_RX_LFC_EN)
        await tb.driver.hw_regs.read_dword(0)

        lfc_xoff = Ether(src='DA:D1:D2:D3:D4:D5', dst='01:80:C2:00:00:01', type=0x8808) / struct.pack('!HH', 0x0001, 2000)

        await tb.sfp_source[0].send(XgmiiFrame.from_payload(bytes(lfc_xoff)))

        count = 16

        pkts = [bytearray([(x+k) % 256 for x in range(1514)]) for k in range(count)]

        tb.loopback_enable = True

        for p in pkts:
            await tb.driver.interfaces[0].ndevs[0].start_xmit(p, 0)

        for k in range(count):
            pkt = await tb.driver.interfaces[0].ndevs[0].recv()

            tb.log.info("Packet: %s", pkt)
            assert pkt.data == pkts[k]
            if tb.driver.interfaces[0].if_feature_rx_csum:
                assert pkt.rx_checksum == ~scapy.utils.checksum(bytes(pkt.data[14:])) & 0xffff

        tb.loopback_enable = False

    await RisingEdge(dut.clk_300mhz)
    await RisingEdge(dut.clk_300mhz)


# cocotb-test

tests_dir = os.path.dirname(__file__)
rtl_dir = os.path.abspath(os.path.join(tests_dir, '..', '..', 'rtl'))
lib_dir = os.path.abspath(os.path.join(rtl_dir, '..', 'lib'))
app_dir = os.path.abspath(os.path.join(rtl_dir, '..', 'app'))
axi_rtl_dir = os.path.abspath(os.path.join(lib_dir, 'axi', 'rtl'))
axis_rtl_dir = os.path.abspath(os.path.join(lib_dir, 'axis', 'rtl'))
eth_rtl_dir = os.path.abspath(os.path.join(lib_dir, 'eth', 'rtl'))
pcie_rtl_dir = os.path.abspath(os.path.join(lib_dir, 'pcie', 'rtl'))


def test_fpga_core(request):
    dut = "fpga_core"
    module = os.path.splitext(os.path.basename(__file__))[0]
    toplevel = dut

    includes = [
        os.path.join(app_dir, "custom_port_demo", "rtl"),
    ]

    verilog_sources = [
        os.path.join(rtl_dir, f"{dut}_custom_port_demo.v"),
        os.path.join(rtl_dir, "common", "mqnic_core_axi.v"),
        os.path.join(rtl_dir, "common", "mqnic_core.v"),
        os.path.join(rtl_dir, "common", "mqnic_dram_if.v"),
        os.path.join(rtl_dir, "common", "mqnic_interface.v"),
        os.path.join(rtl_dir, "common", "mqnic_interface_tx.v"),
        os.path.join(rtl_dir, "common", "mqnic_interface_rx.v"),
        os.path.join(rtl_dir, "common", "mqnic_port.v"),
        os.path.join(rtl_dir, "common", "mqnic_port_tx.v"),
        os.path.join(rtl_dir, "common", "mqnic_port_rx.v"),
        os.path.join(rtl_dir, "common", "mqnic_egress.v"),
        os.path.join(rtl_dir, "common", "mqnic_ingress.v"),
        os.path.join(rtl_dir, "common", "mqnic_l2_egress.v"),
        os.path.join(rtl_dir, "common", "mqnic_l2_ingress.v"),
        os.path.join(rtl_dir, "common", "mqnic_rx_queue_map.v"),
        os.path.join(rtl_dir, "common", "mqnic_ptp.v"),
        os.path.join(rtl_dir, "common", "mqnic_ptp_clock.v"),
        os.path.join(rtl_dir, "common", "mqnic_ptp_perout.v"),
        os.path.join(rtl_dir, "common", "mqnic_rb_clk_info.v"),
        os.path.join(rtl_dir, "common", "mqnic_port_map_phy_xgmii.v"),
        os.path.join(rtl_dir, "common", "cpl_write.v"),
        os.path.join(rtl_dir, "common", "cpl_op_mux.v"),
        os.path.join(rtl_dir, "common", "desc_fetch.v"),
        os.path.join(rtl_dir, "common", "desc_op_mux.v"),
        os.path.join(rtl_dir, "common", "queue_manager.v"),
        os.path.join(rtl_dir, "common", "cpl_queue_manager.v"),
        os.path.join(rtl_dir, "common", "tx_fifo.v"),
        os.path.join(rtl_dir, "common", "rx_fifo.v"),
        os.path.join(rtl_dir, "common", "tx_req_mux.v"),
        os.path.join(rtl_dir, "common", "tx_engine.v"),
        os.path.join(rtl_dir, "common", "rx_engine.v"),
        os.path.join(rtl_dir, "common", "tx_checksum.v"),
        os.path.join(rtl_dir, "common", "rx_hash.v"),
        os.path.join(rtl_dir, "common", "rx_checksum.v"),
        os.path.join(rtl_dir, "common", "rb_drp.v"),
        os.path.join(rtl_dir, "common", "stats_counter.v"),
        os.path.join(rtl_dir, "common", "stats_collect.v"),
        os.path.join(rtl_dir, "common", "stats_dma_if_axi.v"),
        os.path.join(rtl_dir, "common", "stats_dma_latency.v"),
        os.path.join(rtl_dir, "common", "mqnic_tx_scheduler_block_rr.v"),
        os.path.join(rtl_dir, "common", "tx_scheduler_rr.v"),
        os.path.join(rtl_dir, "common", "tdma_scheduler.v"),
        os.path.join(rtl_dir, "common", "tdma_ber.v"),
        os.path.join(rtl_dir, "common", "tdma_ber_ch.v"),
        os.path.join(app_dir, "custom_port_demo", "rtl", "mqnic_app_block_custom_port_demo.v"),
        os.path.join(eth_rtl_dir, "eth_mac_10g.v"),
        os.path.join(eth_rtl_dir, "axis_xgmii_rx_64.v"),
        os.path.join(eth_rtl_dir, "axis_xgmii_tx_64.v"),
        os.path.join(eth_rtl_dir, "mac_ctrl_rx.v"),
        os.path.join(eth_rtl_dir, "mac_ctrl_tx.v"),
        os.path.join(eth_rtl_dir, "mac_pause_ctrl_rx.v"),
        os.path.join(eth_rtl_dir, "mac_pause_ctrl_tx.v"),
        os.path.join(eth_rtl_dir, "lfsr.v"),
        os.path.join(eth_rtl_dir, "ptp_td_phc.v"),
        os.path.join(eth_rtl_dir, "ptp_td_leaf.v"),
        os.path.join(eth_rtl_dir, "ptp_td_rel2tod.v"),
        os.path.join(eth_rtl_dir, "ptp_perout.v"),
        os.path.join(axi_rtl_dir, "axil_interconnect.v"),
        os.path.join(axi_rtl_dir, "axil_crossbar.v"),
        os.path.join(axi_rtl_dir, "axil_crossbar_addr.v"),
        os.path.join(axi_rtl_dir, "axil_crossbar_rd.v"),
        os.path.join(axi_rtl_dir, "axil_crossbar_wr.v"),
        os.path.join(axi_rtl_dir, "axil_reg_if.v"),
        os.path.join(axi_rtl_dir, "axil_reg_if_rd.v"),
        os.path.join(axi_rtl_dir, "axil_reg_if_wr.v"),
        os.path.join(axi_rtl_dir, "axil_register_rd.v"),
        os.path.join(axi_rtl_dir, "axil_register_wr.v"),
        os.path.join(axi_rtl_dir, "arbiter.v"),
        os.path.join(axi_rtl_dir, "priority_encoder.v"),
        os.path.join(axis_rtl_dir, "axis_adapter.v"),
        os.path.join(axis_rtl_dir, "axis_arb_mux.v"),
        os.path.join(axis_rtl_dir, "axis_async_fifo.v"),
        os.path.join(axis_rtl_dir, "axis_async_fifo_adapter.v"),
        os.path.join(axis_rtl_dir, "axis_demux.v"),
        os.path.join(axis_rtl_dir, "axis_fifo.v"),
        os.path.join(axis_rtl_dir, "axis_fifo_adapter.v"),
        os.path.join(axis_rtl_dir, "axis_pipeline_fifo.v"),
        os.path.join(axis_rtl_dir, "axis_register.v"),
        os.path.join(pcie_rtl_dir, "irq_rate_limit.v"),
        os.path.join(pcie_rtl_dir, "dma_if_axi.v"),
        os.path.join(pcie_rtl_dir, "dma_if_axi_rd.v"),
        os.path.join(pcie_rtl_dir, "dma_if_axi_wr.v"),
        os.path.join(pcie_rtl_dir, "dma_if_mux.v"),
        os.path.join(pcie_rtl_dir, "dma_if_mux_rd.v"),
        os.path.join(pcie_rtl_dir, "dma_if_mux_wr.v"),
        os.path.join(pcie_rtl_dir, "dma_if_desc_mux.v"),
        os.path.join(pcie_rtl_dir, "dma_ram_demux_rd.v"),
        os.path.join(pcie_rtl_dir, "dma_ram_demux_wr.v"),
        os.path.join(pcie_rtl_dir, "dma_psdpram.v"),
        os.path.join(pcie_rtl_dir, "dma_client_axis_sink.v"),
        os.path.join(pcie_rtl_dir, "dma_client_axis_source.v"),
        os.path.join(pcie_rtl_dir, "pulse_merge.v"),
    ]

    defines = []
    parameters = {}

    # Structural configuration
    parameters['IF_COUNT'] = 2
    parameters['PORTS_PER_IF'] = 1
    parameters['SCHED_PER_IF'] = parameters['PORTS_PER_IF']
    parameters['PORT_MASK'] = 0

    # Clock configuration
    parameters['CLK_PERIOD_NS_NUM'] = 10
    parameters['CLK_PERIOD_NS_DENOM'] = 3

    # PTP configuration
    parameters['PTP_CLK_PERIOD_NS_NUM'] = 32
    parameters['PTP_CLK_PERIOD_NS_DENOM'] = 5
    parameters['PTP_CLOCK_PIPELINE'] = 0
    parameters['PTP_CLOCK_CDC_PIPELINE'] = 0
    parameters['PTP_PORT_CDC_PIPELINE'] = 0
    parameters['PTP_PEROUT_ENABLE'] = 1
    parameters['PTP_PEROUT_COUNT'] = 1

    # Queue manager configuration
    parameters['EVENT_QUEUE_OP_TABLE_SIZE'] = 32
    parameters['TX_QUEUE_OP_TABLE_SIZE'] = 32
    parameters['RX_QUEUE_OP_TABLE_SIZE'] = 32
    parameters['CQ_OP_TABLE_SIZE'] = 32
    parameters['EQN_WIDTH'] = 2
    parameters['TX_QUEUE_INDEX_WIDTH'] = 5
    parameters['RX_QUEUE_INDEX_WIDTH'] = 5
    parameters['CQN_WIDTH'] = max(parameters['TX_QUEUE_INDEX_WIDTH'], parameters['RX_QUEUE_INDEX_WIDTH']) + 1
    parameters['EQ_PIPELINE'] = 3
    parameters['TX_QUEUE_PIPELINE'] = 3 + max(parameters['TX_QUEUE_INDEX_WIDTH']-12, 0)
    parameters['RX_QUEUE_PIPELINE'] = 3 + max(parameters['RX_QUEUE_INDEX_WIDTH']-12, 0)
    parameters['CQ_PIPELINE'] = 3 + max(parameters['CQN_WIDTH']-12, 0)

    # TX and RX engine configuration
    parameters['TX_DESC_TABLE_SIZE'] = 32
    parameters['RX_DESC_TABLE_SIZE'] = 32
    parameters['RX_INDIR_TBL_ADDR_WIDTH'] = min(parameters['RX_QUEUE_INDEX_WIDTH'], 8)

    # Scheduler configuration
    parameters['TX_SCHEDULER_OP_TABLE_SIZE'] = parameters['TX_DESC_TABLE_SIZE']
    parameters['TX_SCHEDULER_PIPELINE'] = parameters['TX_QUEUE_PIPELINE']
    parameters['TDMA_INDEX_WIDTH'] = 6

    # Interface configuration
    parameters['PTP_TS_ENABLE'] = 1
    parameters['TX_CPL_FIFO_DEPTH'] = 32
    parameters['TX_CHECKSUM_ENABLE'] = 1
    parameters['RX_HASH_ENABLE'] = 1
    parameters['RX_CHECKSUM_ENABLE'] = 1
    parameters['LFC_ENABLE'] = 1
    parameters['PFC_ENABLE'] = parameters['LFC_ENABLE']
    parameters['TX_FIFO_DEPTH'] = 32768
    parameters['RX_FIFO_DEPTH'] = 65536
    parameters['MAX_TX_SIZE'] = 9214
    parameters['MAX_RX_SIZE'] = 9214
    parameters['TX_RAM_SIZE'] = 32768
    parameters['RX_RAM_SIZE'] = 32768

    # Application block configuration
    parameters['APP_ID'] = 0x12349001
    parameters['APP_ENABLE'] = 1
    parameters['APP_CTRL_ENABLE'] = 1
    parameters['APP_DMA_ENABLE'] = 1
    parameters['APP_AXIS_DIRECT_ENABLE'] = 1
    parameters['APP_AXIS_SYNC_ENABLE'] = 1
    parameters['APP_AXIS_IF_ENABLE'] = 1
    parameters['APP_STAT_ENABLE'] = 1

    # Custom ports and parameters
    defines.append("APP_CUSTOM_PARAMS_ENABLE")
    defines.append("APP_CUSTOM_PORTS_ENABLE")

    parameters['AXIL_APP_CUSTOM_DATA_WIDTH'] = 32

    # AXI DMA interface configuration
    parameters['AXI_DATA_WIDTH'] = 128
    parameters['AXI_ADDR_WIDTH'] = 40
    parameters['AXI_ID_WIDTH'] = 4

    # DMA interface configuration
    parameters['DMA_IMM_ENABLE'] = 0
    parameters['DMA_IMM_WIDTH'] = 32
    parameters['DMA_LEN_WIDTH'] = 16
    parameters['DMA_TAG_WIDTH'] = 16
    parameters['RAM_ADDR_WIDTH'] = (max(parameters['TX_RAM_SIZE'], parameters['RX_RAM_SIZE'])-1).bit_length()
    parameters['RAM_PIPELINE'] = 2
    parameters['AXI_DMA_MAX_BURST_LEN'] = 16

    # AXI lite interface configuration (control)
    parameters['AXIL_CTRL_DATA_WIDTH'] = 32
    parameters['AXIL_CTRL_ADDR_WIDTH'] = 24

    # AXI lite interface configuration (application control)
    parameters['AXIL_APP_CTRL_DATA_WIDTH'] = parameters['AXIL_CTRL_DATA_WIDTH']
    parameters['AXIL_APP_CTRL_ADDR_WIDTH'] = 24

    # Ethernet interface configuration
    parameters['AXIS_ETH_TX_PIPELINE'] = 0
    parameters['AXIS_ETH_TX_FIFO_PIPELINE'] = 2
    parameters['AXIS_ETH_TX_TS_PIPELINE'] = 0
    parameters['AXIS_ETH_RX_PIPELINE'] = 0
    parameters['AXIS_ETH_RX_FIFO_PIPELINE'] = 2

    # Statistics counter subsystem
    parameters['STAT_ENABLE'] = 1
    parameters['STAT_DMA_ENABLE'] = 1
    parameters['STAT_AXI_ENABLE'] = 1
    parameters['STAT_INC_WIDTH'] = 24
    parameters['STAT_ID_WIDTH'] = 12

    extra_env = {f'PARAM_{k}': str(v) for k, v in parameters.items()}

    sim_build = os.path.join(tests_dir, "sim_build",
        request.node.name.replace('[', '-').replace(']', ''))

    cocotb_test.simulator.run(
        python_search=[tests_dir],
        verilog_sources=verilog_sources,
        toplevel=toplevel,
        module=module,
        includes=includes,
        defines=defines,
        parameters=parameters,
        sim_build=sim_build,
        extra_env=extra_env,
    )

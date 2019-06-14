#include "pim/pim_kernel.hh"

#include <cassert>
#include <typeinfo>

#include "base/callback.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "mem/dram_ctrl.hh"
#include "mem/request.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

#define PIM_SG

PIMKernel::PIMKernel(const Params *p)
    : MemObject(p), port(this), mem_port(name() + ".mem_port", *this),
      status(Idle), parms(p), tickEvent([this] { tick(); }, "PIMKernel tick",
                                        false, Event::Default_Pri),
      computeEvent([this] { trycompute(); }, "PIMKernel compute", false,
                   Event::Default_Pri),
      finishEvent([this] { finish(); }, "PIMKernel finish data process", false,
                  Event::Default_Pri),
      _id(p->id), _latency(p->latency), _input(p->input), _output(p->output),
      pim_addr_base(p->addr_base),
      addrRanges(p->addr_ranges.begin(), p->addr_ranges.end()) {
  std::cout << p->addr_ranges.begin()->start();
  for (int i = 0; i < p->input + p->output; i++) {
    regs.push_back(make_pair<PIMKernel::dataType, DataStatus>(0, dataEmpty));
    data.push_back(0);
  }
  raw_data = new uint8_t[p->input];
  DPRINTF(PIM, "Kernel: %d [0x%lx - 0x%lx]\n", _id,
          addrRanges.begin()->start(), addrRanges.end()->end());
}

void PIMKernel::trycompute() {
  dataType res = doCompute();
  int toProc = locateLatest();
  DPRINTF(PIM, "Start processing data write of Reg[%d]\n", toProc);
  Request::Flags flags = 0;
  RequestPtr req = new Request(regs[toProc].first, size, flags, 0);

  PacketPtr pkt = new Packet(req, MemCmd::PIMWrite);
  uint8_t *empty = new uint8_t[size];

  memcpy(empty, &res, sizeof(dataType));

  pkt->dataDynamic(empty);

  Packet::PIMSenderState *senderState = new Packet::PIMSenderState(_id,
                                                                   toProc);
  pkt->pushSenderState(senderState);

  DPRINTF(PIM, "Try to send write req [0x%llx]-[%d]\n", regs[toProc].first,
          toProc);

  if (port.sendTimingReq(pkt)) {
    status = WaitingResp;

    DPRINTF(PIM, "Writen to the memory [0x%llx]-[%d] [%d]\n",
            regs[toProc].first, toProc, status);

    regs[toProc].second = dataWaitingResp;
    req = nullptr;
    write_packets++;
  } else {
    DPRINTF(PIM, "Failed sending write to the memory [%d]\n", toProc);
    retry_pkt = pkt;
    status = SendRetry;
    write_retry++;

    if (!computeEvent.scheduled()) {
      schedule(computeEvent, clockEdge((Cycles)1));
    }
  }
}

BaseMasterPort &PIMKernel::getMasterPort(const std::string &if_name,
                                         PortID idx) {
  return port;
}

void PIMKernel::init() {
  if (!mem_port.isConnected())
    fatal("PIM ports on %s are not connected\n", name());

  if (!port.isConnected())
    fatal("PIM ports on %s are not connected\n", name());

  mem_port.sendRangeChange();
}

BaseSlavePort &PIMKernel::getSlavePort(const string &if_name, PortID idx) {
  return mem_port;
}

void PIMKernel::PIMMasterPort::PIMTickEvent::process() {}

bool PIMKernel::recvTimingReq(PacketPtr pkt) {
  //@PIM currently, the control flow of the PIM kernel uses
  // recvFunctional function.
  DPRINTF(PIM, "Get data request\n");
  assert(pkt->getSize() == size * num);
  if (status == Finish) {
    DPRINTF(PIM, "Return data\n");
    Tick request_time = clockEdge((Cycles)1) + pkt->headerDelay;
    pkt->setData(raw_data);
    pkt->makeTimingResponse();
    pkt->headerDelay = pkt->payloadDelay = 0;
    mem_port.schedTimingResp(pkt, request_time, true);
    return true;
  } else
    return false;
}

bool PIMKernel::PIMMasterPort::recvTimingResp(PacketPtr pkt) {
  return kernel->recvTimingResp(pkt);
}

void PIMKernel::recvFunctional(PacketPtr pkt) {
  recv_pim_commands++;
  // receive pim commands from the host
  Packet::PIMSenderState *senderState =
      dynamic_cast<Packet::PIMSenderState *>(pkt->senderState);
  auto *state = new Packet::PIMSenderState(_id, -1);
  for (int i = 0; i < senderState->addr.size(); i++) {
    state->addr.push_back(senderState->addr[i]);
  }

  state->setCommand(Packet::PIMSenderState::Command::Registration);
  Request::Flags flags = 0;
  uint8_t dummy_size = 1;
  RequestPtr req = new Request(pim_addr_base - 1, dummy_size, flags, 0);
  PacketPtr _pkt = new Packet(req, MemCmd::PIM);

  uint8_t *empty = new uint8_t[dummy_size];
  _pkt->dataDynamic(empty);

  _pkt->pushSenderState(state);

  // send pim addresses to the memory to block other memory accesses to the
  // same region
  port.sendFunctional(_pkt);
  sent_pim_commands++;

  if (!isActive() || canModify()) {
    // if a kernel is off or idle, the kernel can be used for computing
    exec_pim_commands++;
    start(pkt);
    return;
  } else {
    failed_pim_commands++;
  }
}

PIMKernel::RecvPIMPort::RecvPIMPort(const std::string &name,
                                    PIMKernel &_kernel)
    : QueuedSlavePort(name, &_kernel, queue), queue(_kernel, *this),
      kernel(_kernel), sendRetryEvent([this] { processSendRetry(); }, name) {}

AddrRangeList PIMKernel::RecvPIMPort::getAddrRanges() const {
  return kernel.addrRanges;
}

void PIMKernel::RecvPIMPort::recvFunctional(PacketPtr pkt) {
  pkt->pushLabel(kernel.name());

  if (!queue.checkFunctional(pkt)) {
    // Default implementation of SimpleTimingPort::recvFunctional()
    // calls recvAtomic() and throws away the latency; we can save a
    // little here by just not calculating the latency.
    kernel.recvFunctional(pkt);
  }

  pkt->popLabel();
}

Tick PIMKernel::RecvPIMPort::recvAtomic(PacketPtr pkt) {
  return kernel.recvAtomic(pkt);
}

bool PIMKernel::RecvPIMPort::recvTimingReq(PacketPtr pkt) {
  // pass it to the memory controller
  bool success =  kernel.recvTimingReq(pkt);
  if (!success)
    owner.schedule(sendRetryEvent, curTick() + 1);
  return success;
}

void PIMKernel::RecvPIMPort::processSendRetry() {
  DPRINTF(PIM, "Port is sending retry\n");

  // reset the flag and call retry
  //mustSendRetry = false;
  sendRetryReq();
}

void PIMKernel::recvReqRetry() {
  retry_cycle++;
  assert(status == SendRetry && retry_pkt != nullptr);
  Packet::PIMSenderState *senderState =
      dynamic_cast<Packet::PIMSenderState *>(retry_pkt->senderState);
  assert(senderState);
  if (port.sendTimingReq(retry_pkt)) {
    status = WaitingResp;
    DPRINTF(PIM, "Resend to memory [0x%llx]-[%d]\n",
            regs[senderState->procid].first, senderState->procid);

    regs[senderState->procid].second = dataWaitingResp;
    if (retry_pkt->isRead()) {
      read_retry++;
    } else {
      write_retry++;
    }
  } else {
    retry_failed++;
    DPRINTF(PIM, "Fail to resend to memory %d\n", senderState->procid);
  }
  if (needSchedule() && !tickEvent.scheduled() && !finishEvent.scheduled())
    schedule(tickEvent, clockEdge((Cycles)1));
}

void PIMKernel::PIMMasterPort::recvReqRetry() { kernel->recvReqRetry(); }

void PIMKernel::TimingPIMPort::TickEvent::schedule(PacketPtr _pkt, Tick t) {
  pkt = _pkt;
  kernel->schedule(this, t);
}

bool PIMKernel::recvTimingResp(PacketPtr pkt) {
  bool res = doDataCallback(pkt, curTick());
  if (!tickEvent.scheduled())
    // delay processing of returned data until next CPU clock edge
    schedule(tickEvent, clockEdge(Cycles(1)));

  return res;
}

void PIMKernel::recvTimingSnoopReq(PacketPtr pkt) {
  warn("recvTimingSnoopReq is not working in PIM kernel");
}

Tick PIMKernel::recvAtomic(PacketPtr pkt) {
  fatal("PIM kernel should not get atomic operations");
  return 0;
}

void PIMKernel::functionalAccess(PacketPtr pkt) {
  warn("functionalAccess is not working in PIM kernel");
}

PIMKernel::~PIMKernel() {}

PIMKernel *PIMKernelParams::create() { return new PIMKernel(this); }

bool PIMKernel::getReadyStatus() {
  // check the kernel has processed all required data
  // sanity check of registers
  assert(regs.size() > 0);
  //return inputReady() && outputReady();
  return inputReady();
}

bool PIMKernel::inputReady() {
  // sanity check of the registers
  assert(_input >= 0);
  for (int i = 0; i < _input; i++) {
    //if (regs[i].second != dataFinish) {
    //  return false;
    //}
    if (regs[i].second != dataFinish && regs[i].second != dataEmpty)
      return false;
  }
  return true;
}

bool PIMKernel::outputReady() {
  // sanity check of the registers
  assert(_output >= 0);
  for (int i = 0; i < _input + _output; i++) {
    if (regs[i].second != dataFinish) {
      return false;
    }
  }
  return true;
}

void PIMKernel::tick() {
  // tick() should work when the kernel is active
  assert(isActive());
  active_cycle++;
  switch (status) {
  case Status::SendRetry:

    DPRINTF(PIM, "Resend memory access packet\n");
    break;

  case Status::Ready:
  case Status::WaitingResp: {
    //if (recPackets.size() == _input + _output - 1) {
    //  for (int i = 0; i < regs.size(); i++) {
    //    regs[i].second = dataFinish;
    //  }
    //  while (recPackets.size() > 0) {
    //    auto f = recPackets.begin();
    //    delete (*f);
    //    recPackets.erase(recPackets.begin());
    //  }
    //}

    int toProc = locateLatest();
    if (toProc < _input) {
      DPRINTF(PIM, "Start processing data read of Reg[%d]\n", toProc);

      Request::Flags flags = 0;
      RequestPtr req = new Request(regs[toProc].first, size, flags, 0);
      PacketPtr pkt = new Packet(req, MemCmd::PIMRead);
      uint8_t *empty = new uint8_t[size];
      pkt->dataDynamic(empty);
      Packet::PIMSenderState *senderState =
          new Packet::PIMSenderState(_id, toProc);
      pkt->pushSenderState(senderState);

      if (port.sendTimingReq(pkt)) {
        read_packets++;
        status = WaitingResp;
        DPRINTF(PIM, "Send to the memory [0x%llx] - [%d]\n",
                regs[toProc].first, toProc);

        regs[toProc].second = dataWaitingResp;
      } else {
        DPRINTF(PIM, "Fail to send to the memory %d\n", toProc);
        retry_pkt = pkt;
        status = SendRetry;
        read_retry++;
      }
    }
    //if (toProc >= _input && toProc < _input + _output && inputReady()) {
    //  if (!computeEvent.scheduled()) {
    //    schedule(computeEvent, clockEdge((Cycles)_latency));
    //  }
    //} else if (inputReady() && outputReady()) {
    //  status = Finish;
    //  DPRINTF(PIM, "Finished PIM oprations\n");
    //}
    //if (inputReady() && status != Ready) {
    //  status = Finish;
    //}
  } break;

  case Status::Idle:
  case Status::Finish: {
    DPRINTF(PIM, "Finish PIM opration\n");
    assert(!finishEvent.scheduled());
    schedule(finishEvent, clockEdge((Cycles)1));

  } break;

  case Status::Poweroff: // doing nothing
  default:
    break;
  }

  if (needSchedule() && !tickEvent.scheduled() && !finishEvent.scheduled()) {
    schedule(tickEvent, clockEdge((Cycles)1));
  }
}

bool PIMKernel::start(PacketPtr pkt) {
  Packet::PIMSenderState *senderState =
      dynamic_cast<Packet::PIMSenderState *>(pkt->senderState);
  assert(senderState);
  addrs = senderState->addr;
  for (int i = 0; i < senderState->addr.size(); i++) {
    regs[i].first = senderState->addr[i];
    regs[i].second = addrReady;
  }
  for (int i = senderState->addr.size(); i < _input; i++)
    regs[i].second = dataEmpty;
  size = pkt->getSize();
  num = senderState->addr.size();
  DPRINTF(PIM, "Size = %d, Num = %d\n", size, num);
  tickid = senderState->cycle;
  active();
  return true;
}

void PIMKernel::active() {
  status = Ready;
  tick();
}

bool PIMKernel::canModify() { return status == Idle; }

bool PIMKernel::powerOff() {
  if (status == Idle || status == Poweroff) {
    status = Poweroff;
    return true;
  }
  return false;
}

bool PIMKernel::isActive() { return status != Poweroff; }

bool PIMKernel::needSchedule() {
  if (status == Idle || status == Poweroff || status == SendRetry)
    return false;
  return true;
}

int PIMKernel::locateLatest() {
  assert(isActive());
  for (int i = 0; i < _input + _output; i++) {
    if (regs[i].second == addrReady)
      return i;
  }

  return _input + _output;
}

bool PIMKernel::readReg(int index) {
  assert(index >= 0 && index < _input + _output);
  if (index < _input)
    return true;
  else
    return false;
}

// process when receiving data callback from the memory
bool PIMKernel::doDataCallback(PacketPtr pkt, Tick response_time) {
  Packet::PIMSenderState *senderState =
      dynamic_cast<Packet::PIMSenderState *>(pkt->senderState);
  assert(senderState);
  int i = senderState->procid;
  assert(i < _output + _input);
  if (size == 4) {
    data[i] = *pkt->getPtr<int32_t>();
    ((int32_t *)raw_data)[i] = *pkt->getPtr<int32_t>();
  } else {
    assert(size == 8);
    data[i] = *pkt->getPtr<int64_t>();
    ((int64_t *)raw_data)[i] = *pkt->getPtr<int64_t>();
  }

  regs[i].second = dataFinish;
  //if (getReadyStatus())
  if (inputReady())
    status = Finish;
  DPRINTF(PIM, "Receive [0x%llx] [%d]- %lld [%d] : status [%d]\n",
          pkt->getAddr(), i, data[i], regs[i].second, status);
  //recPackets.push_back(pkt);
  delete pkt;
  return true;
}

// send pim finish command to the memory to remove pending address
void PIMKernel::finish() {
  assert(!tickEvent.scheduled());

  Packet::PIMSenderState *state = new Packet::PIMSenderState(addrs);
  state->setCommand(Packet::PIMSenderState::Command::Complete);
  Request::Flags flags = 0;
  uint8_t dummy_size = 1;
  RequestPtr req = new Request(pim_addr_base - 1, dummy_size, flags, 0);
  PacketPtr _pkt = new Packet(req, MemCmd::PIM);
  uint8_t *empty = new uint8_t[dummy_size];
  _pkt->dataDynamic(empty);
  _pkt->pushSenderState(state);
  port.sendFunctional(_pkt);

  status = Idle;
}

void PIMKernel::regStats() {
  MemObject::regStats();
  using namespace Stats;

  recv_pim_commands.name(name() + ".recv_pim_commands")
      .desc("The PIM command received from the host-side processor");

  computing_counts.name(name() + ".computing_counts")
      .desc("the counts of the computing progress");

  sent_pim_commands.name(name() + ".sent_pim_commands")
      .desc("the control commands sent by pim kernel");

  read_packets.name(name() + ".read_packets")
      .desc("the amount of pim kernel read");

  write_packets.name(name() + ".write_packets")
      .desc("the amount of pim kernel write");

  read_retry.name(name() + ".read_retry")
      .desc("the amount of pim kernel read retry");

  write_retry.name(name() + ".write_retry")
      .desc("the amount of pim kernel write retry");

  retry_failed.name(name() + ".retry_failed")
      .desc("the amount of failed pim kernel requests");

  active_cycle.name(name() + ".active_cycle")
      .desc("the active cycles of the kernel");

  retry_cycle.name(name() + ".retry_cycle")
      .desc("the retry cycles of the kernel");
}

PIMKernel::dataType PIMKernel::doCompute() {
  warn("basic PIM kernel has no computing progress.");
  computing_counts++;
  return (dataType)0;
}

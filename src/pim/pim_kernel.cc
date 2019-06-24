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
      _id(p->id), _latency(p->latency), _input(p->input), _output(p->output),
      pim_addr_base(p->addr_base),
      addrRanges(p->addr_ranges.begin(), p->addr_ranges.end()) {
  for (int i = 0; i < p->input + p->output; i++) {
    regs.push_back(make_pair<PIMKernel::dataType, DataStatus>(0, dataEmpty));
    data.push_back(0);
  }
  raw_data = new uint8_t[p->input];
  DPRINTF(PIM, "Kernel: %d [0x%lx - 0x%lx]\n", _id,
          addrRanges.begin()->start(), addrRanges.end()->end());
}

void PIMKernel::init() {
  if (!mem_port.isConnected())
    fatal("PIM ports on %s are not connected\n", name());

  if (!port.isConnected())
    fatal("PIM ports on %s are not connected\n", name());

  mem_port.sendRangeChange();
}

bool PIMKernel::recvTimingReq(PacketPtr pkt) {
  DPRINTF(PIM, "Get data request\n");
  if (status == Finish) {
    //assert(pkt->getSize() == size * num);
    //auto *state = pkt->senderState;
    //while (state->predecessor)
    //  state = state->predecessor;
    //auto *senderState = dynamic_cast<Packet::PIMSenderState *>(state);
    //assert(senderState);
    //if (senderState->addr.size() != addrs.size()) {
    //  DPRINTF(PIM, "ERR: size does not match\n");
    //  return false;
    //}
    //for (int i = 0; i < senderState->addr.size(); i++) {
    //  if (senderState->addr[i] != addrs[i]) {
    //    DPRINTF(PIM, "ERR: address does not match %d\n", i);
    //    return false;
    //  }
    //}
    DPRINTF(PIM, "Return data\n");
    Tick request_time = clockEdge((Cycles)1) + pkt->headerDelay;
    pkt->setSize(size * num);
    pkt->setData(raw_data);
    pkt->makeTimingResponse();
    pkt->headerDelay = pkt->payloadDelay = 0;
    mem_port.schedTimingResp(pkt, request_time, true);
    status = Idle;
    return true;
  } else
    return false;
}

void PIMKernel::recvFunctional(PacketPtr pkt) {
  // receive pim commands from the host
  recv_pim_commands++;
  // send pim addresses to the memory to block other memory accesses to the
  // same region
  Packet::PIMSenderState *senderState =
      dynamic_cast<Packet::PIMSenderState *>(pkt->senderState);
  auto *state = new Packet::PIMSenderState(_id, senderState->addr, -1);
  state->setCommand(Packet::PIMSenderState::Command::Registration);
  Request::Flags flags = 0;
  uint8_t dummy_size = 1;
  RequestPtr req = new Request(pim_addr_base - 1, dummy_size, flags, 0);
  PacketPtr _pkt = new Packet(req, MemCmd::PIM);
  uint8_t *empty = new uint8_t[dummy_size];
  _pkt->dataDynamic(empty);
  _pkt->pushSenderState(state);
  port.sendFunctional(_pkt);
  sent_pim_commands++;

  // Put pkt in a queue if it cannot be served this cycle.
  funcQueue.push_back(pkt);

  if (!tickEvent.scheduled())
    schedule(tickEvent, clockEdge(Cycles(1)));
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

bool PIMKernel::isReady() {
  // sanity check of the registers
  assert(_input >= 0);
  for (int i = 0; i < _input; i++) {
    if (regs[i].second != dataFinish && regs[i].second != dataEmpty)
      return false;
  }
  return true;
}

void PIMKernel::tick() {
  // tick() should work when the kernel is active
  assert(isActive());
  active_cycle++;
  switch (status) {
  case Status::SendRetry:
    break;

  case Status::Ready:
  case Status::WaitingResp: {
    int toProc = locateLatest();
    if (toProc < _input) {
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
  } break;

  case Status::Idle:
    start();
    break;
  case Status::Finish: {
    if (!finished) {
      DPRINTF(PIM, "Finish PIM opration\n");
      finish();
      finished = true;
    }
    mem_port.retryTimingReq();
  } break;

  case Status::Poweroff: // doing nothing
  default:
    break;
  }

  if (needSchedule() && !tickEvent.scheduled()) {
    schedule(tickEvent, clockEdge((Cycles)1));
  }
}

void PIMKernel::start() {
  if (funcQueue.empty())
    return;

  PacketPtr pkt = funcQueue.front();
  Packet::PIMSenderState *senderState =
      dynamic_cast<Packet::PIMSenderState *>(pkt->senderState);
  assert(senderState);
  addrs = senderState->addr;
  for (int i = 0; i < senderState->addr.size(); i++) {
    regs[i].first = senderState->addr[i];
    // Distinguish empty requests.
    if (senderState->addr[i])
      regs[i].second = addrReady;
    else
      regs[i].second = dataEmpty;
  }
  for (int i = senderState->addr.size(); i < _input; i++)
    regs[i].second = dataEmpty;
  size = pkt->getSize();
  num = senderState->addr.size();
  DPRINTF(PIM, "Start: size = %d, num = %d\n", size, num);
  status = Ready;
  finished = false;
  funcQueue.erase(funcQueue.begin());
}

bool PIMKernel::powerOff() {
  if (status == Idle || status == Poweroff) {
    status = Poweroff;
    return true;
  }
  return false;
}

bool PIMKernel::isActive() { return status != Poweroff; }

bool PIMKernel::needSchedule() {
  if (status == Poweroff || status == SendRetry)
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

  assert(regs[i].second == addrReady ||
         regs[i].second == dataWaitingResp);
  regs[i].second = dataFinish;
  if (isReady())
    status = Finish;
  DPRINTF(PIM, "Receive [0x%llx] [%d]- %lld [%d] : status [%d]\n",
          pkt->getAddr(), i, data[i], regs[i].second, status);
  delete pkt;
  return true;
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
  }
  if (needSchedule() && !tickEvent.scheduled())
    schedule(tickEvent, clockEdge((Cycles)1));
}

// send pim finish command to the memory to remove pending address
void PIMKernel::finish() {
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

BaseMasterPort &PIMKernel::getMasterPort(const std::string &if_name,
                                         PortID idx) {
  return port;
}

BaseSlavePort &PIMKernel::getSlavePort(const string &if_name, PortID idx) {
  return mem_port;
}

PIMKernel::RecvPIMPort::RecvPIMPort(const std::string &name,
                                    PIMKernel &_kernel)
    : QueuedSlavePort(name, &_kernel, queue), queue(_kernel, *this),
      kernel(_kernel), sendRetryEvent([this] { processSendRetry(); }, name),
      blockedNum(0) {}

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

void PIMKernel::RecvPIMPort::retryTimingReq() {
  if (blockedNum) {
    assert(!sendRetryEvent.scheduled());
    owner.schedule(sendRetryEvent, kernel.clockEdge((Cycles)1));
    blockedNum--;
  }
}

bool PIMKernel::RecvPIMPort::recvTimingReq(PacketPtr pkt) {
  if (blockedNum) {
    blockedNum++;
    return false;
  }
  bool success = kernel.recvTimingReq(pkt);
  if (!success)
    blockedNum = 1;
  return success;
}

void PIMKernel::RecvPIMPort::processSendRetry() {
  DPRINTF(PIM, "Port is sending retry\n");

  // reset the flag and call retry
  //mustSendRetry = false;
  sendRetryReq();
}

void PIMKernel::PIMMasterPort::PIMTickEvent::process() {}

bool PIMKernel::PIMMasterPort::recvTimingResp(PacketPtr pkt) {
  return kernel->recvTimingResp(pkt);
}

void PIMKernel::PIMMasterPort::recvReqRetry() { kernel->recvReqRetry(); }

void PIMKernel::TimingPIMPort::TickEvent::schedule(PacketPtr _pkt, Tick t) {
  pkt = _pkt;
  kernel->schedule(this, t);
}

PIMKernel *PIMKernelParams::create() { return new PIMKernel(this); }

package openLLC

import chisel3._
import chisel3.util._
import coupledL2.tl2chi._
import org.chipsalliance.cde.config.Parameters
import utility.FastArbiter

class PrefetchState(implicit  p: Parameters) extends LLCBundle{
  val w_datRsp = Bool()     // wait for rsp from memory
  val w_queryReq = Bool()   // wait for be queried (hit?)
}

class PrefetchEntry(implicit p: Parameters) extends TaskEntry{
  val state = new PrefetchState()
  val data = new DSBlock()
  val query_task = new Task()
  val beatValids = Vec(beatSize, Bool())
}

class PrefetchRequest(implicit p: Parameters) extends LLCBundle{
  val state = new PrefetchState()
  val task = new Task()
}

class PrefetchQuery(implicit p: Parameters) extends LLCBundle{
  /* Query to check whether is in PrefetchBuffer */
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  val task = new Task()
}

class PrefetchQueryResult(implicit p: Parameters) extends LLCBundle{
  val hit = Bool()
}

class PrefetchInfo(implicit p: Parameters) extends BlockInfo{

}

class PrefetchUnit(implicit p: Parameters) extends LLCModule with HasCHIOpcodes{
  val io = IO(new Bundle(){
    /* PrefetchTgt from MainPipe */
    val fromMainPipe = new Bundle(){
      val alloc_s4 = Flipped(ValidIO(new PrefetchRequest()))
    }

    /* Response from downstream memory */
    val snRxdat = Flipped(ValidIO(new RespWithData()))

    /* Bypass dat from MemUnit (fake CompData) */
    val bypassData = Flipped(Vec(beatSize, ValidIO(new RespWithData())))

    /* Query whether hit prefetched cachelines */
    val query_req = Flipped(ValidIO(new PrefetchQuery()))   // s2
    val query_rsp  = ValidIO(new PrefetchQueryResult())     // s3

    /* Send data to ResponseUnit */
    val toRespUnit = Vec(beatSize, ValidIO(new RespWithData()))

    /* Prefetch buffers info */
    //val prefetchInfo = Vec(mshrs.prefetch, ValidIO(new PrefetchInfo()))
  })

  val alloc_s4    = io.fromMainPipe.alloc_s4
  val memData     = io.snRxdat
  val bypassData  = io.bypassData
  val query_req   = io.query_req
  val query_rsp   = io.query_rsp
  val to_rsp_unit = io.toRespUnit

  /* Data Structure */
  val buffer = RegInit(VecInit(Seq.fill(mshrs.prefetch)(0.U.asTypeOf(new PrefetchEntry()))))
  val txdatArb = Module(new FastArbiter(new TaskWithData(), mshrs.prefetch))

  /* Query */
  val hit_vec_s2 = buffer.map(e =>
    e.valid && e.task.tag === query_req.bits.tag && e.task.set === query_req.bits.set && query_req.valid
  )
  assert(PopCount(hit_vec_s2) < 2.U, "Prefetch Task repeated")
  val hit_s2      = Cat(hit_vec_s2).orR
  val hit_id_s2   = PriorityEncoder(hit_vec_s2)

  val hit_s3        = RegNext(hit_s2, false.B)
  val hit_id_s3     = RegNext(hit_id_s2, false.B)
  val query_req_s3  = RegNext(query_req, 0.U.asTypeOf(query_req))

  when(hit_s3) {
    buffer(hit_id_s3).state.w_queryReq := true.B
    buffer(hit_id_s3).query_task  := query_req_s3.bits.task
  }

  query_rsp.valid := query_req_s3.valid
  query_rsp.bits.hit := hit_s3

  /* Alloc */
  val freeVec_s4 = VecInit(buffer.map(!_.valid))
  val idOH_s4 = PriorityEncoderOH(freeVec_s4)
  val insertIdx_s4 = OHToUInt(idOH_s4)

  val full_s4 = !(Cat(freeVec_s4).orR)
  val canAlloc_s4 = alloc_s4.valid && !full_s4

  when(canAlloc_s4) {
    val entry = buffer(insertIdx_s4)
    entry.valid := true.B
    entry.task := alloc_s4.bits.task
    entry.state := alloc_s4.bits.state
    entry.beatValids := VecInit(Seq.fill(beatSize)(false.B))
  }

  // TODO: add PLRU for reallocate the buffer entry
  assert(!(full_s4 && alloc_s4.valid), "PrefetchBuf overflow")

  /* Update State */
  def handleMemResp(response: Valid[RespWithData], isBypass: Boolean): Unit = {
    when(response.valid) {
      val update_vec = buffer.map(e =>
        e.task.reqID === response.bits.txnID && e.valid && response.bits.opcode === CompData && !e.state.w_datRsp
      )
      assert(PopCount(update_vec) < 2.U, "Response task repeated")
      val canUpdate = Cat(update_vec).orR
      val update_id = PriorityEncoder(update_vec)
      when(canUpdate) {
        val entry = buffer(update_id)
        val beatId = response.bits.dataID >> log2Ceil(beatBytes / 16)
        entry.state.w_datRsp := { if (!isBypass) PopCount(entry.beatValids) === (beatSize - 1).U else true.B }
        entry.beatValids(beatId) := true.B
        entry.data.data(beatId) := response.bits.data
      }
    }
  }

  handleMemResp(memData, isBypass = false)
  for (i <- 0 until beatSize) {
    handleMemResp(bypassData(i), isBypass = true)
  }

  /* Issue */
  val isPrefetch = buffer.map(e => e.task.chiOpcode === PrefetchTgt)
  for (i <- 0 until mshrs.prefetch) {
    txdatArb.io.in(i).valid     := buffer(i).valid && buffer(i).state.w_datRsp && buffer(i).state.w_queryReq && isPrefetch(i)
    txdatArb.io.in(i).bits.task := buffer(i).query_task
    txdatArb.io.in(i).bits.data := buffer(i).data
  }
  txdatArb.io.out.ready := true.B

  /*
    Timeline of a load req:
      s2: query
      s3: hit
      s4: alloc in ResponseUnit, and the data in PrefetchUnit may be ready to issue
      s5: PrefetchUnit (may) provide data
   */
  val txdatArb_out_reg = RegNext(txdatArb.io.out.bits, 0.U.asTypeOf(txdatArb.io.out.bits))
  for (i <- 0 until beatSize) {
    val rsp_with_data = WireInit(0.U.asTypeOf(new RespWithData()))
    rsp_with_data.txnID   := txdatArb_out_reg.task.txnID
    rsp_with_data.opcode  := CompData
    rsp_with_data.data    := txdatArb_out_reg.data.data(i)
    rsp_with_data.dataID  := (beatBytes * i * 8).U(log2Ceil(blockBytes * 8) - 1, log2Ceil(blockBytes * 8) - 2)

    to_rsp_unit(i).valid  := RegNext(txdatArb.io.out.valid, false.B)
    to_rsp_unit(i).bits   := rsp_with_data
  }

  /* Dealloc */
  val will_free_vec = buffer.map(e =>
    e.valid && e.state.w_datRsp && e.state.w_queryReq
  )
  for (i <- 0 until mshrs.prefetch) {
    when(will_free_vec(i)) {
      buffer(i).valid := false.B
    }
  }


  /* Performance Counter */
  if (cacheParams.enablePerf) {
    val bufferTimer = RegInit(VecInit(Seq.fill(mshrs.prefetch)(0.U(16.W))))
    buffer.zip(bufferTimer).zipWithIndex.map { case ((e, t), i) =>
      when(e.valid && !e.state.w_datRsp) { t := t + 1.U }
      when(RegNext(e.valid && !e.state.w_datRsp, false.B) && !(e.valid && !e.state.w_datRsp)) { t := 0.U }
      assert(t < timeoutThreshold.U, "PrefetchBuf Leak(id: %d)", i.U)
    }
  }
}
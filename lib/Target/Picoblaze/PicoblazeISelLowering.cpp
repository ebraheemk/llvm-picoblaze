//===-- PicoblazeISelLowering.cpp - Picoblaze DAG Lowering Implementation  ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the PicoblazeTargetLowering class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Picoblaze-lower"
#include "llvm/codegen/PseudoSourceValue.h"
#include "PicoblazeISelLowering.h"
#include "Picoblaze.h"
#include "PicoblazeMachineFunctionInfo.h"
#include "PicoblazeTargetMachine.h"
#include "PicoblazeSubtarget.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Function.h"
#include "llvm/Intrinsics.h"
#include "llvm/CallingConv.h"
#include "llvm/GlobalVariable.h"
#include "llvm/GlobalAlias.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

typedef enum {
  NoHWMult,
  HWMultIntr,
  HWMultNoIntr
} HWMultUseMode;

static cl::opt<HWMultUseMode>
HWMultMode("Picoblaze-hwmult-mode",
           cl::desc("Hardware multiplier use mode"),
           cl::init(HWMultNoIntr),
           cl::values(
             clEnumValN(NoHWMult, "no",
                "Do not use hardware multiplier"),
             clEnumValN(HWMultIntr, "interrupts",
                "Assume hardware multiplier can be used inside interrupts"),
             clEnumValN(HWMultNoIntr, "use",
                "Assume hardware multiplier cannot be used inside interrupts"),
             clEnumValEnd));

PicoblazeTargetLowering::PicoblazeTargetLowering(PicoblazeTargetMachine &tm) :
  TargetLowering(tm, new TargetLoweringObjectFileELF()),
  Subtarget(*tm.getSubtargetImpl()) {
  
  PR_FUNCTION();

  TD = getDataLayout();

  // Set up the register classes.
  // 我们只支持8位的操作
  addRegisterClass(MVT::i8,  &Picoblaze::GR8RegClass);
//  addRegisterClass(MVT::i16,  &Picoblaze::GR16RegClass);
  

  // Compute derived properties from the register classes
  computeRegisterProperties();

  // Provide all sorts of operation actions

  // Division is expensive
  setIntDivIsCheap(false);

  setStackPointerRegisterToSaveRestore(Picoblaze::SP);

  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrOneBooleanContent); // FIXME: Is this correct?

/*
   enum LegalizeAction {
     Legal,      // The target natively supports this operation.
     Promote,    // This operation should be executed in a larger type.
     Expand,     // Try to expand this to other ops, otherwise use a libcall.
     Custom      // Use the LowerOperation hook to implement custom lowering.
   };
*/

  setLoadExtAction(ISD::EXTLOAD,  MVT::i1,  Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i1,  Promote);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i1,  Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i8,  Expand);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i16,  Expand);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i32,  Expand);

  
  setOperationAction(ISD::FP_TO_SINT, MVT::i16, Custom);
  setOperationAction(ISD::FP_TO_UINT, MVT::i16, Custom);

  //setOperationAction(ISD::LOAD, MVT::i8, Custom);
 // setOperationAction(ISD::STORE, MVT::i8, Custom);

  // We don't have any truncstores
  //setTruncStoreAction(MVT::i16, MVT::i8, Expand);

  setOperationAction(ISD::SRA,              MVT::i8,    Custom);
  setOperationAction(ISD::SHL,              MVT::i8,    Custom);
  setOperationAction(ISD::SRL,              MVT::i8,    Custom);
  setOperationAction(ISD::ROTL,             MVT::i8,    Expand);
  setOperationAction(ISD::ROTR,             MVT::i8,    Expand);
  setOperationAction(ISD::BR_JT,            MVT::Other, Expand);
  setOperationAction(ISD::BR_CC,            MVT::i8,    Custom);
  setOperationAction(ISD::BRCOND,           MVT::Other, Expand);
  setOperationAction(ISD::SETCC,            MVT::i8,    Custom);
  setOperationAction(ISD::SELECT,           MVT::i8,    Expand);
  setOperationAction(ISD::SELECT_CC,        MVT::i8,    Custom);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i8, Expand);

  setOperationAction(ISD::CTTZ,             MVT::i8,    Expand);
  setOperationAction(ISD::CTTZ_ZERO_UNDEF,  MVT::i8,    Expand);
  setOperationAction(ISD::CTLZ,             MVT::i8,    Expand);
  setOperationAction(ISD::CTLZ_ZERO_UNDEF,  MVT::i8,    Expand);
  setOperationAction(ISD::CTPOP,            MVT::i8,    Expand);

  setOperationAction(ISD::SHL_PARTS,        MVT::i8,    Expand);
  setOperationAction(ISD::SRL_PARTS,        MVT::i8,    Expand);
  setOperationAction(ISD::SRA_PARTS,        MVT::i8,    Expand);

  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1,   Expand);

  // FIXME: Implement efficiently multiplication by a constant
  setOperationAction(ISD::MUL,              MVT::i8,    Expand);
  setOperationAction(ISD::MULHS,            MVT::i8,    Expand);
  setOperationAction(ISD::MULHU,            MVT::i8,    Expand);
  setOperationAction(ISD::SMUL_LOHI,        MVT::i8,    Expand);
  setOperationAction(ISD::UMUL_LOHI,        MVT::i8,    Expand);
  setOperationAction(ISD::UDIV,             MVT::i8,    Expand);
  setOperationAction(ISD::UDIVREM,          MVT::i8,    Expand);
  setOperationAction(ISD::UREM,             MVT::i8,    Expand);
  setOperationAction(ISD::SDIV,             MVT::i8,    Expand);
  setOperationAction(ISD::SDIVREM,          MVT::i8,    Expand);
  setOperationAction(ISD::SREM,             MVT::i8,    Expand);


  setOperationAction(ISD::GlobalAddress,    MVT::i8,   Custom);
  setOperationAction(ISD::GlobalAddress,    MVT::i16,   Custom);
  setOperationAction(ISD::ExternalSymbol,    MVT::i16,   Custom);
  setOperationAction(ISD::ExternalSymbol,    MVT::i8,   Custom);
  //setOperationAction(ISD::TargetGlobalAddress,    MVT::i16,   Custom);
  setOperationAction(ISD::FrameIndex,    MVT::i8,   Legal);

  // Libcalls names.
 /* if (HWMultMode == HWMultIntr) {
    setLibcallName(RTLIB::MUL_I8,  "__mulqi3hw");
    setLibcallName(RTLIB::MUL_I16, "__mulhi3hw");
  } else if (HWMultMode == HWMultNoIntr) {
    setLibcallName(RTLIB::MUL_I8,  "__mulqi3hw_noint");
    setLibcallName(RTLIB::MUL_I16, "__mulhi3hw_noint");
  }
  */
  setMinFunctionAlignment(0);
  setPrefFunctionAlignment(0);
}

SDValue PicoblazeTargetLowering::LowerOperation(SDValue Op,
                                             SelectionDAG &DAG) const 
{
 PR_FUNCTION();
  switch (Op.getOpcode()) {
  case ISD::SHL: // FALLTHROUGH
  case ISD::SRL:
  case ISD::SRA:              return LowerShifts(Op, DAG);
  case ISD::GlobalAddress:    return LowerGlobalAddress(Op, DAG);
  case ISD::BlockAddress:     return LowerBlockAddress(Op, DAG);
  case ISD::ExternalSymbol:   return LowerExternalSymbol(Op, DAG);
  case ISD::SETCC:            return LowerSETCC(Op, DAG);
  case ISD::BR_CC:            return LowerBR_CC(Op, DAG);
  case ISD::SELECT_CC:        return LowerSELECT_CC(Op, DAG);
  case ISD::SIGN_EXTEND:      return LowerSIGN_EXTEND(Op, DAG);
  case ISD::RETURNADDR:       return LowerRETURNADDR(Op, DAG);
  case ISD::FRAMEADDR:        return LowerFRAMEADDR(Op, DAG);
  case ISD::LOAD  :
  case ISD::STORE  :
      return LowerLoadStore(Op,DAG);
  case ISD::FrameIndex:
	  {
		  return LowerFrameIndex(Op,DAG);
	  }
 case PicoblazeISD::CALL:
	  {
		SDValue vop1,vop2,vret;
		DebugLoc dl=Op.getDebugLoc();
		Op->getNumValues();
		vop1 = DAG.getConstant(0,Op->getValueType(0));
		vop2 = DAG.getNode(PicoblazeISD::PBP,dl,Op->getValueType(0));
		return vret = DAG.getNode(PicoblazeISD::CALL,dl,Op->getValueType(0),vop1,vop2);
			  
	  }
	  break;
  default:
	  DAG.viewGraph();
    llvm_unreachable("unimplemented operand");
  }
}

//===----------------------------------------------------------------------===//
//                       Picoblaze Inline Assembly Support
//===----------------------------------------------------------------------===//

/// getConstraintType - Given a constraint letter, return the type of
/// constraint it is for this target.
TargetLowering::ConstraintType
PicoblazeTargetLowering::getConstraintType(const std::string &Constraint) const {
PR_FUNCTION();
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    case 'r':
      return C_RegisterClass;
    default:
      break;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

std::pair<unsigned, const TargetRegisterClass*>
PicoblazeTargetLowering::
getRegForInlineAsmConstraint(const std::string &Constraint,
                             EVT VT) const {
								 PR_FUNCTION();
  if (Constraint.size() == 1) {
    // GCC Constraint Letters
    switch (Constraint[0]) {
    default: break;
    case 'r':   // GENERAL_REGS
        return std::make_pair(0U, &Picoblaze::GR8RegClass);
    }
  }

  return TargetLowering::getRegForInlineAsmConstraint(Constraint, VT);
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

#include "PicoblazeGenCallingConv.inc"

SDValue
PicoblazeTargetLowering::LowerFormalArguments(SDValue Chain,
                                           CallingConv::ID CallConv,
                                           bool isVarArg,
                                           const SmallVectorImpl<ISD::InputArg>
                                             &Ins,
                                           DebugLoc dl,
                                           SelectionDAG &DAG,
                                           SmallVectorImpl<SDValue> &InVals)
                                             const {

												 PR_FUNCTION();
  switch (CallConv) {
  default:
    llvm_unreachable("Unsupported calling convention");
  case CallingConv::C:
  case CallingConv::Fast:
    return LowerCCCArguments(Chain, CallConv, isVarArg, Ins, dl, DAG, InVals);
  //case CallingConv::Picoblaze_INTR:
  //  if (Ins.empty())
  //    return Chain;
    report_fatal_error("ISRs cannot have arguments");
  }
}

SDValue
PicoblazeTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                                SmallVectorImpl<SDValue> &InVals) const {
									PR_FUNCTION();
  SelectionDAG &DAG                     = CLI.DAG;
  DebugLoc &dl                          = CLI.DL;
  SmallVector<ISD::OutputArg, 32> &Outs = CLI.Outs;
  SmallVector<SDValue, 32> &OutVals     = CLI.OutVals;
  SmallVector<ISD::InputArg, 32> &Ins   = CLI.Ins;
  SDValue Chain                         = CLI.Chain;
  SDValue Callee                        = CLI.Callee;
  bool &isTailCall                      = CLI.IsTailCall;
  CallingConv::ID CallConv              = CLI.CallConv;
  bool isVarArg                         = CLI.IsVarArg;

  // Picoblaze target does not yet support tail call optimization.
  isTailCall = false;

  switch (CallConv) {
  default:
    llvm_unreachable("Unsupported calling convention");
  case CallingConv::Fast:
  case CallingConv::C:
    return LowerCCCCallTo(Chain, Callee, CallConv, isVarArg, isTailCall,
                          Outs, OutVals, Ins, dl, DAG, InVals);
  //case CallingConv::Picoblaze_INTR:
  //  report_fatal_error("ISRs cannot be called directly");
  }
}

/// LowerCCCArguments - transform physical registers into virtual registers and
/// generate load operations for arguments places on the stack.
// FIXME: struct return stuff
// FIXME: varargs
SDValue
PicoblazeTargetLowering::LowerCCCArguments(SDValue Chain,
                                        CallingConv::ID CallConv,
                                        bool isVarArg,
                                        const SmallVectorImpl<ISD::InputArg>
                                          &Ins,
                                        DebugLoc dl,
                                        SelectionDAG &DAG,
                                        SmallVectorImpl<SDValue> &InVals)
                                          const {
											  PR_FUNCTION();
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo *MFI = MF.getFrameInfo();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                 getTargetMachine(), ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeFormalArguments(Ins, CC_Picoblaze);

  assert(!isVarArg && "Varargs not supported yet");

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    if (VA.isRegLoc()) {
      // Arguments passed in registers
      EVT RegVT = VA.getLocVT();
      switch (RegVT.getSimpleVT().SimpleTy) {
      default:
        {
#ifndef NDEBUG
          errs() << "LowerFormalArguments Unhandled argument type: "
               << RegVT.getSimpleVT().SimpleTy << "\n";
#endif
          llvm_unreachable(0);
        }
      case MVT::i8:
        unsigned VReg = RegInfo.createVirtualRegister(&Picoblaze::GR8RegClass);
        RegInfo.addLiveIn(VA.getLocReg(), VReg);
        SDValue ArgValue = DAG.getCopyFromReg(Chain, dl, VReg, RegVT);

        // If this is an 8-bit value, it is really passed promoted to 16
        // bits. Insert an assert[sz]ext to capture this, then truncate to the
        // right size.
        if (VA.getLocInfo() == CCValAssign::SExt)
          ArgValue = DAG.getNode(ISD::AssertSext, dl, RegVT, ArgValue,
                                 DAG.getValueType(VA.getValVT()));
        else if (VA.getLocInfo() == CCValAssign::ZExt)
          ArgValue = DAG.getNode(ISD::AssertZext, dl, RegVT, ArgValue,
                                 DAG.getValueType(VA.getValVT()));

        if (VA.getLocInfo() != CCValAssign::Full)
          ArgValue = DAG.getNode(ISD::TRUNCATE, dl, VA.getValVT(), ArgValue);

        InVals.push_back(ArgValue);
      

      }
    } else {
      // Sanity check
      assert(VA.isMemLoc());
      // Load the argument to a virtual register
      unsigned ObjSize = VA.getLocVT().getSizeInBits()/8;
      if (ObjSize > 2) {
        errs() << "LowerFormalArguments Unhandled argument type: "
             << EVT(VA.getLocVT()).getEVTString()
             << "\n";
      }
      // Create the frame index object for this incoming parameter...
      int FI = MFI->CreateFixedObject(ObjSize, VA.getLocMemOffset(), true);

      // Create the SelectionDAG nodes corresponding to a load
      //from this parameter
      SDValue FIN = DAG.getFrameIndex(FI, MVT::i8);
      InVals.push_back(DAG.getLoad(VA.getLocVT(), dl, Chain, FIN,
                                   MachinePointerInfo::getFixedStack(FI),
                                   false, false, false, 0));
    }
  }

  return Chain;
}

SDValue
PicoblazeTargetLowering::LowerReturn(SDValue Chain,
                                  CallingConv::ID CallConv, bool isVarArg,
                                  const SmallVectorImpl<ISD::OutputArg> &Outs,
                                  const SmallVectorImpl<SDValue> &OutVals,
                                  DebugLoc dl, SelectionDAG &DAG) const {
									  PR_FUNCTION();

  // CCValAssign - represent the assignment of the return value to a location
  SmallVector<CCValAssign, 16> RVLocs;

  // ISRs cannot return any value.
 // if (CallConv == CallingConv::Picoblaze_INTR && !Outs.empty())
 //   report_fatal_error("ISRs cannot return any value");

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                 getTargetMachine(), RVLocs, *DAG.getContext());

  // Analize return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_Picoblaze);

  // If this is the first return lowered for this function, add the regs to the
  // liveout set for the function.
  if (DAG.getMachineFunction().getRegInfo().liveout_empty()) {
    for (unsigned i = 0; i != RVLocs.size(); ++i)
      if (RVLocs[i].isRegLoc())
        DAG.getMachineFunction().getRegInfo().addLiveOut(RVLocs[i].getLocReg());
  }

  SDValue Flag;

  // Copy the result values into the output registers.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, dl, VA.getLocReg(),
                             OutVals[i], Flag);

    // Guarantee that all emitted copies are stuck together,
    // avoiding something bad.
    Flag = Chain.getValue(1);
  }

  unsigned Opc = 
                  PicoblazeISD::RET_FLAG;

  if (Flag.getNode())
    return DAG.getNode(Opc, dl, MVT::Other, Chain, Flag);

  // Return Void
  return DAG.getNode(Opc, dl, MVT::Other, Chain);
}

/// LowerCCCCallTo - functions arguments are copied from virtual regs to
/// (physical regs)/(stack frame), CALLSEQ_START and CALLSEQ_END are emitted.
/// TODO: sret.
SDValue
PicoblazeTargetLowering::LowerCCCCallTo(SDValue Chain, SDValue Callee,
                                     CallingConv::ID CallConv, bool isVarArg,
                                     bool isTailCall,
                                     const SmallVectorImpl<ISD::OutputArg>
                                       &Outs,
                                     const SmallVectorImpl<SDValue> &OutVals,
                                     const SmallVectorImpl<ISD::InputArg> &Ins,
                                     DebugLoc dl, SelectionDAG &DAG,
                                     SmallVectorImpl<SDValue> &InVals) const {
										 PR_FUNCTION();
  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                 getTargetMachine(), ArgLocs, *DAG.getContext());

  CCInfo.AnalyzeCallOperands(Outs, CC_Picoblaze);

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = CCInfo.getNextStackOffset();

  Chain = DAG.getCALLSEQ_START(Chain ,DAG.getConstant(NumBytes,
                                                      getPointerTy(), true));

  SmallVector<std::pair<unsigned, SDValue>, 4> RegsToPass;
  SmallVector<SDValue, 12> MemOpChains;
  SDValue StackPtr;

  // Walk the register/memloc assignments, inserting copies/loads.
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];

    SDValue Arg = OutVals[i];

    // Promote the value if needed.
    switch (VA.getLocInfo()) {
      default: llvm_unreachable("Unknown loc info!");
      case CCValAssign::Full: break;
      case CCValAssign::SExt:
        Arg = DAG.getNode(ISD::SIGN_EXTEND, dl, VA.getLocVT(), Arg);
        break;
      case CCValAssign::ZExt:
        Arg = DAG.getNode(ISD::ZERO_EXTEND, dl, VA.getLocVT(), Arg);
        break;
      case CCValAssign::AExt:
        Arg = DAG.getNode(ISD::ANY_EXTEND, dl, VA.getLocVT(), Arg);
        break;
    }

    // Arguments that can be passed on register must be kept at RegsToPass
    // vector
    if (VA.isRegLoc()) {
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
    } else {
      assert(VA.isMemLoc());

      if (StackPtr.getNode() == 0)
        StackPtr = DAG.getCopyFromReg(Chain, dl, Picoblaze::SP, getPointerTy());

      SDValue PtrOff = DAG.getNode(ISD::ADD, dl, getPointerTy(),
                                   StackPtr,
                                   DAG.getIntPtrConstant(VA.getLocMemOffset()));


      MemOpChains.push_back(DAG.getStore(Chain, dl, Arg, PtrOff,
                                         MachinePointerInfo(),false, false, 0));
    }
  }

  // Transform all store nodes into one single node because all store nodes are
  // independent of each other.
  if (!MemOpChains.empty())
    Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other,
                        &MemOpChains[0], MemOpChains.size());

  // Build a sequence of copy-to-reg nodes chained together with token chain and
  // flag operands which copy the outgoing args into registers.  The InFlag in
  // necessary since all emitted instructions must be stuck together.
  SDValue InFlag;
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = DAG.getCopyToReg(Chain, dl, RegsToPass[i].first,
                             RegsToPass[i].second, InFlag);
    InFlag = Chain.getValue(1);
  }

  // If the callee is a GlobalAddress node (quite common, every direct call is)
  // turn it into a TargetGlobalAddress node so that legalize doesn't hack it.
  // Likewise ExternalSymbol -> TargetExternalSymbol.

  if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee))
  {
    Callee = DAG.getGlobalAddress(G->getGlobal(), dl, MVT::i8);
  }
  else if (ExternalSymbolSDNode *E = dyn_cast<ExternalSymbolSDNode>(Callee))
  {
	  Callee = DAG.getExternalSymbol(E->getSymbol(), MVT::i8);
  }
  // Returns a chain & a flag for retval copy to use.
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
    Ops.push_back(DAG.getRegister(RegsToPass[i].first,
                                  RegsToPass[i].second.getValueType()));

  if (InFlag.getNode())
    Ops.push_back(InFlag);

  Chain = DAG.getNode(PicoblazeISD::CALL, dl, NodeTys, &Ops[0], Ops.size());

  InFlag = Chain.getValue(1);

  // Create the CALLSEQ_END node.
  Chain = DAG.getCALLSEQ_END(Chain,
                             DAG.getConstant(NumBytes, getPointerTy(), true),
                             DAG.getConstant(0, getPointerTy(), true),
                             InFlag);
  InFlag = Chain.getValue(1);

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InFlag, CallConv, isVarArg, Ins, dl,
                         DAG, InVals);
}

/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
///
SDValue
PicoblazeTargetLowering::LowerCallResult(SDValue Chain, SDValue InFlag,
                                      CallingConv::ID CallConv, bool isVarArg,
                                      const SmallVectorImpl<ISD::InputArg> &Ins,
                                      DebugLoc dl, SelectionDAG &DAG,
                                      SmallVectorImpl<SDValue> &InVals) const {
										  PR_FUNCTION();

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(),
                 getTargetMachine(), RVLocs, *DAG.getContext());

  CCInfo.AnalyzeCallResult(Ins, RetCC_Picoblaze);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned i = 0; i != RVLocs.size(); ++i) {
    Chain = DAG.getCopyFromReg(Chain, dl, RVLocs[i].getLocReg(),
                               RVLocs[i].getValVT(), InFlag).getValue(1);
    InFlag = Chain.getValue(2);
    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}

SDValue PicoblazeTargetLowering::LowerShifts(SDValue Op,
                                          SelectionDAG &DAG) const {
											  PR_FUNCTION();
  unsigned Opc = Op.getOpcode();
  SDNode* N = Op.getNode();
  EVT VT = Op.getValueType();
  DebugLoc dl = N->getDebugLoc();

  // Expand non-constant shifts to loops:
  if (!isa<ConstantSDNode>(N->getOperand(1)))
    switch (Opc) {
    default: llvm_unreachable("Invalid shift opcode!");
    case ISD::SHL:
      return DAG.getNode(PicoblazeISD::SHL, dl,
                         VT, N->getOperand(0), N->getOperand(1));
    case ISD::SRA:
      return DAG.getNode(PicoblazeISD::SRA, dl,
                         VT, N->getOperand(0), N->getOperand(1));
    case ISD::SRL:
      return DAG.getNode(PicoblazeISD::SRL, dl,
                         VT, N->getOperand(0), N->getOperand(1));
    }

  uint64_t ShiftAmount = cast<ConstantSDNode>(N->getOperand(1))->getZExtValue();

  // Expand the stuff into sequence of shifts.
  // FIXME: for some shift amounts this might be done better!
  // E.g.: foo >> (8 + N) => sxt(swpb(foo)) >> N
  SDValue Victim = N->getOperand(0);

  if (Opc == ISD::SRL && ShiftAmount) {
    // Emit a special goodness here:
    // srl A, 1 => clrc; rrc A
    Victim = DAG.getNode(PicoblazeISD::RRC, dl, VT, Victim);
    ShiftAmount -= 1;
  }

  while (ShiftAmount--)
    Victim = DAG.getNode((Opc == ISD::SHL ? PicoblazeISD::RLA : PicoblazeISD::RRA),
                         dl, VT, Victim);

  return Victim;
}

SDValue PicoblazeTargetLowering::LowerGlobalAddress(SDValue Op,
                                                 SelectionDAG &DAG) const {
													 PR_FUNCTION();
  const GlobalValue *GV = cast<GlobalAddressSDNode>(Op)->getGlobal();
  int64_t Offset = cast<GlobalAddressSDNode>(Op)->getOffset();

  // Create the TargetGlobalAddress node, folding in the constant offset.
  SDValue Result = DAG.getTargetGlobalAddress(GV, Op.getDebugLoc(),
                                              getPointerTy(), Offset);
  return DAG.getNode(PicoblazeISD::Wrapper, Op.getDebugLoc(),
                     getPointerTy(), Result);
}

SDValue PicoblazeTargetLowering::LowerExternalSymbol(SDValue Op,
                                                  SelectionDAG &DAG) const {
													  PR_FUNCTION();
  DebugLoc dl = Op.getDebugLoc();
  const char *Sym = cast<ExternalSymbolSDNode>(Op)->getSymbol();
  SDValue Result = DAG.getTargetExternalSymbol(Sym, getPointerTy());

  return DAG.getNode(PicoblazeISD::Wrapper, dl, getPointerTy(), Result);
}

SDValue PicoblazeTargetLowering::LowerBlockAddress(SDValue Op,
                                                SelectionDAG &DAG) const {
													PR_FUNCTION();
  DebugLoc dl = Op.getDebugLoc();
  const BlockAddress *BA = cast<BlockAddressSDNode>(Op)->getBlockAddress();
  SDValue Result = DAG.getTargetBlockAddress(BA, getPointerTy());

  return DAG.getNode(PicoblazeISD::Wrapper, dl, getPointerTy(), Result);
}

static SDValue EmitCMP(SDValue &LHS, SDValue &RHS, SDValue &TargetCC,
                       ISD::CondCode CC,
                       DebugLoc dl, SelectionDAG &DAG) {
  // FIXME: Handle bittests someday
  assert(!LHS.getValueType().isFloatingPoint() && "We don't handle FP yet");

  // FIXME: Handle jump negative someday
  PicoblazeCC::CondCodes TCC = PicoblazeCC::COND_INVALID;
  switch (CC) {
  default: llvm_unreachable("Invalid integer condition!");
  case ISD::SETEQ:
    TCC = PicoblazeCC::COND_E;     // aka COND_Z
    // Minor optimization: if LHS is a constant, swap operands, then the
    // constant can be folded into comparison.
    if (LHS.getOpcode() == ISD::Constant)
      std::swap(LHS, RHS);
    break;
  case ISD::SETNE:
    TCC = PicoblazeCC::COND_NE;    // aka COND_NZ
    // Minor optimization: if LHS is a constant, swap operands, then the
    // constant can be folded into comparison.
    if (LHS.getOpcode() == ISD::Constant)
      std::swap(LHS, RHS);
    break;
  case ISD::SETULE:
    std::swap(LHS, RHS);        // FALLTHROUGH
  case ISD::SETUGE:
    // Turn lhs u>= rhs with lhs constant into rhs u< lhs+1, this allows us to
    // fold constant into instruction.
    if (const ConstantSDNode * C = dyn_cast<ConstantSDNode>(LHS)) {
      LHS = RHS;
      RHS = DAG.getConstant(C->getSExtValue() + 1, C->getValueType(0));
      TCC = PicoblazeCC::COND_LO;
      break;
    }
    TCC = PicoblazeCC::COND_HS;    // aka COND_C
    break;
  case ISD::SETUGT:
    std::swap(LHS, RHS);        // FALLTHROUGH
  case ISD::SETULT:
    // Turn lhs u< rhs with lhs constant into rhs u>= lhs+1, this allows us to
    // fold constant into instruction.
    if (const ConstantSDNode * C = dyn_cast<ConstantSDNode>(LHS)) {
      LHS = RHS;
      RHS = DAG.getConstant(C->getSExtValue() + 1, C->getValueType(0));
      TCC = PicoblazeCC::COND_HS;
      break;
    }
    TCC = PicoblazeCC::COND_LO;    // aka COND_NC
    break;
  case ISD::SETLE:
    std::swap(LHS, RHS);        // FALLTHROUGH
  case ISD::SETGE:
    // Turn lhs >= rhs with lhs constant into rhs < lhs+1, this allows us to
    // fold constant into instruction.
    if (const ConstantSDNode * C = dyn_cast<ConstantSDNode>(LHS)) {
      LHS = RHS;
      RHS = DAG.getConstant(C->getSExtValue() + 1, C->getValueType(0));
      TCC = PicoblazeCC::COND_L;
      break;
    }
    TCC = PicoblazeCC::COND_GE;
    break;
  case ISD::SETGT:
    std::swap(LHS, RHS);        // FALLTHROUGH
  case ISD::SETLT:
    // Turn lhs < rhs with lhs constant into rhs >= lhs+1, this allows us to
    // fold constant into instruction.
    if (const ConstantSDNode * C = dyn_cast<ConstantSDNode>(LHS)) {
      LHS = RHS;
      RHS = DAG.getConstant(C->getSExtValue() + 1, C->getValueType(0));
      TCC = PicoblazeCC::COND_GE;
      break;
    }
    TCC = PicoblazeCC::COND_L;
    break;
  }

  TargetCC = DAG.getConstant(TCC, MVT::i8);
  return DAG.getNode(PicoblazeISD::CMP, dl, MVT::Glue, LHS, RHS);
}


SDValue PicoblazeTargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
	PR_FUNCTION();
  SDValue Chain = Op.getOperand(0);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(1))->get();
  SDValue LHS   = Op.getOperand(2);
  SDValue RHS   = Op.getOperand(3);
  SDValue Dest  = Op.getOperand(4);
  DebugLoc dl   = Op.getDebugLoc();

  SDValue TargetCC;
  SDValue Flag = EmitCMP(LHS, RHS, TargetCC, CC, dl, DAG);

  return DAG.getNode(PicoblazeISD::BR_CC, dl, Op.getValueType(),
                     Chain, Dest, TargetCC, Flag);
}

SDValue PicoblazeTargetLowering::LowerSETCC(SDValue Op, SelectionDAG &DAG) const {
	PR_FUNCTION();
  SDValue LHS   = Op.getOperand(0);
  SDValue RHS   = Op.getOperand(1);
  DebugLoc dl   = Op.getDebugLoc();

  // If we are doing an AND and testing against zero, then the CMP
  // will not be generated.  The AND (or BIT) will generate the condition codes,
  // but they are different from CMP.
  // FIXME: since we're doing a post-processing, use a pseudoinstr here, so
  // lowering & isel wouldn't diverge.
  bool andCC = false;
  if (ConstantSDNode *RHSC = dyn_cast<ConstantSDNode>(RHS)) {
    if (RHSC->isNullValue() && LHS.hasOneUse() &&
        (LHS.getOpcode() == ISD::AND ||
         (LHS.getOpcode() == ISD::TRUNCATE &&
          LHS.getOperand(0).getOpcode() == ISD::AND))) {
      andCC = true;
    }
  }
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();
  SDValue TargetCC;
  SDValue Flag = EmitCMP(LHS, RHS, TargetCC, CC, dl, DAG);

  // Get the condition codes directly from the status register, if its easy.
  // Otherwise a branch will be generated.  Note that the AND and BIT
  // instructions generate different flags than CMP, the carry bit can be used
  // for NE/EQ.
  bool Invert = false;
  bool Shift = false;
  bool Convert = true;
  switch (cast<ConstantSDNode>(TargetCC)->getZExtValue()) {
   default:
    Convert = false;
    break;
   case PicoblazeCC::COND_HS:
     // Res = SRW & 1, no processing is required
     break;
   case PicoblazeCC::COND_LO:
     // Res = ~(SRW & 1)
     Invert = true;
     break;
   case PicoblazeCC::COND_NE:
     if (andCC) {
       // C = ~Z, thus Res = SRW & 1, no processing is required
     } else {
       // Res = ~((SRW >> 1) & 1)
       Shift = true;
       Invert = true;
     }
     break;
   case PicoblazeCC::COND_E:
     Shift = true;
     // C = ~Z for AND instruction, thus we can put Res = ~(SRW & 1), however,
     // Res = (SRW >> 1) & 1 is 1 word shorter.
     break;
  }
  EVT VT = Op.getValueType();
  SDValue One  = DAG.getConstant(1, VT);
  if (Convert) {
    SDValue SR = DAG.getCopyFromReg(DAG.getEntryNode(), dl, Picoblaze::SP,
                                    MVT::i8, Flag);
    if (Shift)
      // FIXME: somewhere this is turned into a SRL, lower it MSP specific?
      SR = DAG.getNode(ISD::SRA, dl, MVT::i16, SR, One);
    SR = DAG.getNode(ISD::AND, dl, MVT::i16, SR, One);

    return SR;
  } else {
    SDValue Zero = DAG.getConstant(0, VT);
    SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
    SmallVector<SDValue, 4> Ops;
    Ops.push_back(One);
    Ops.push_back(Zero);
    Ops.push_back(TargetCC);
    Ops.push_back(Flag);
    return DAG.getNode(PicoblazeISD::SELECT_CC, dl, VTs, &Ops[0], Ops.size());
  }
}

SDValue PicoblazeTargetLowering::LowerSELECT_CC(SDValue Op,
                                             SelectionDAG &DAG) const {
												 PR_FUNCTION();
  SDValue LHS    = Op.getOperand(0);
  SDValue RHS    = Op.getOperand(1);
  SDValue TrueV  = Op.getOperand(2);
  SDValue FalseV = Op.getOperand(3);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(4))->get();
  DebugLoc dl    = Op.getDebugLoc();

  SDValue TargetCC;
  SDValue Flag = EmitCMP(LHS, RHS, TargetCC, CC, dl, DAG);

  SDVTList VTs = DAG.getVTList(Op.getValueType(), MVT::Glue);
  SmallVector<SDValue, 4> Ops;
  Ops.push_back(TrueV);
  Ops.push_back(FalseV);
  Ops.push_back(TargetCC);
  Ops.push_back(Flag);

  return DAG.getNode(PicoblazeISD::SELECT_CC, dl, VTs, &Ops[0], Ops.size());
}

SDValue PicoblazeTargetLowering::LowerSIGN_EXTEND(SDValue Op,
                                               SelectionDAG &DAG) const {
												   PR_FUNCTION();
  SDValue Val = Op.getOperand(0);
  EVT VT      = Op.getValueType();
  DebugLoc dl = Op.getDebugLoc();

  assert(VT == MVT::i16 && "Only support i16 for now!");

  return DAG.getNode(ISD::SIGN_EXTEND_INREG, dl, VT,
                     DAG.getNode(ISD::ANY_EXTEND, dl, VT, Val),
                     DAG.getValueType(Val.getValueType()));
}

SDValue
PicoblazeTargetLowering::getReturnAddressFrameIndex(SelectionDAG &DAG) const {
	PR_FUNCTION();
  MachineFunction &MF = DAG.getMachineFunction();
  PicoblazeMachineFunctionInfo *FuncInfo = MF.getInfo<PicoblazeMachineFunctionInfo>();
  int ReturnAddrIndex = FuncInfo->getRAIndex();

  if (ReturnAddrIndex == 0) {
    // Set up a frame object for the return address.
    uint64_t SlotSize = TD->getPointerSize();
    ReturnAddrIndex = MF.getFrameInfo()->CreateFixedObject(SlotSize, -SlotSize,
                                                           true);
    FuncInfo->setRAIndex(ReturnAddrIndex);
  }

  return DAG.getFrameIndex(ReturnAddrIndex, getPointerTy());
}

SDValue PicoblazeTargetLowering::LowerRETURNADDR(SDValue Op,
                                              SelectionDAG &DAG) const {
												  PR_FUNCTION();
  MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
  MFI->setReturnAddressIsTaken(true);

  unsigned Depth = cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue();
  DebugLoc dl = Op.getDebugLoc();

  if (Depth > 0) {
    SDValue FrameAddr = LowerFRAMEADDR(Op, DAG);
    SDValue Offset =
      DAG.getConstant(TD->getPointerSize(), MVT::i16);
    return DAG.getLoad(getPointerTy(), dl, DAG.getEntryNode(),
                       DAG.getNode(ISD::ADD, dl, getPointerTy(),
                                   FrameAddr, Offset),
                       MachinePointerInfo(), false, false, false, 0);
  }

  // Just load the return address.
  SDValue RetAddrFI = getReturnAddressFrameIndex(DAG);
  return DAG.getLoad(getPointerTy(), dl, DAG.getEntryNode(),
                     RetAddrFI, MachinePointerInfo(), false, false, false, 0);
}

SDValue PicoblazeTargetLowering::LowerLoadStore(SDValue Op,
                                             SelectionDAG &DAG) const 
{
	
	PR_FUNCTION();

	const StoreSDNode *ST=dyn_cast<const StoreSDNode>(Op);
	if( 1)//ST->isIndexed())
	{
		// indexed store
		DebugLoc dl = Op.getDebugLoc();
		SDValue op0;
		SDValue op1;
		SDValue op2;
		SDValue op3;
		op0= ST->getOperand(0);
		op1= ST->getOperand(1);
		op2= ST->getOperand(2);
		op3= ST->getOperand(3);
		const MachinePointerInfo &mpi=ST->getPointerInfo();
		SDValue empty;
		if( (      (mpi.Offset==0) 
			    || (mpi.V ==NULL)
			)
			&&
			(
		       ( op2->getOpcode()==ISD::UNDEF)
			   ||( op3->getOpcode()==ISD::UNDEF)
		     )
			)
		{
			return empty;
		}
		 unsigned STAlign = ST->getAlignment();
		 SDValue basePtr=ST->getBasePtr();
		 SDValue offset = ST->getOffset();
		 SDValue sdNew;
		if(  (mpi.V!=NULL)
			//&&( PseudoSourceValue::classof(mpi.V ) )
			)
		{
			if( op2->getOpcode()==ISD::UNDEF)
			
			{
				sdNew= DAG.getStore( op0,dl,
						op1,
						op3,
						MachinePointerInfo(NULL,0), false, false, STAlign
						);

			}else if( op3->getOpcode()==ISD::UNDEF)
			{
			  MachineFunction &MF = DAG.getMachineFunction();
			  unsigned Flags = MachineMemOperand::MOStore;
			  if (false)
				Flags |= MachineMemOperand::MOVolatile;
			  if (false)
				Flags |= MachineMemOperand::MONonTemporal;
			   
			  MachineMemOperand *MMO =
				 MF.getMachineMemOperand(MachinePointerInfo(NULL,0), Flags,
                            op1.getValueType().getStoreSize(), STAlign,
                            NULL);

				sdNew= DAG.getStore( op0,dl,
						op1,
						op2,
						MMO
						);

			}else
			{
				sdNew= DAG.getStore( op0,dl,
					op1,
					DAG.getNode(ISD::ADD, dl, MVT::i8,op2,op3),
					MachinePointerInfo(NULL,0), false, false, STAlign
					);

			}
			DAG.viewGraph();
			if(sdNew.getNode()!= Op.getNode())
				return sdNew;
			else
				return empty;
		}
		if( 
	       ( op2->getOpcode()==ISD::UNDEF)
		   ||( op3->getOpcode()==ISD::UNDEF)
		   
		   )
		{
			return empty;
		}else
		{
			DAG.viewGraph();
		   

		 sdNew= DAG.getStore( op0,dl,
			op1,
			DAG.getNode(ISD::ADD, dl, MVT::i8,op2,op3),
			MachinePointerInfo(NULL,0), false, false, STAlign);
		   DAG.viewGraph();
		}
		 return sdNew;
					
	}else
	{
		SDValue empty;
		return empty;
	}
	
	
}

SDValue PicoblazeTargetLowering::LowerFrameIndex(SDValue Op,
                                             SelectionDAG &DAG) const {
												 PR_FUNCTION();
	SDValue vop1,vop2,vret;
	DebugLoc dl=Op.getDebugLoc();
	Op->getNumValues();
	vop1 = DAG.getConstant(0,Op->getValueType(0));
	vop2 = DAG.getNode(PicoblazeISD::PBP,dl,Op->getValueType(0));
	vret = DAG.getNode(ISD::ADD,dl,Op->getValueType(0),vop1,vop2);
	
	
	return vret;
}

SDValue PicoblazeTargetLowering::LowerFRAMEADDR(SDValue Op,
                                             SelectionDAG &DAG) const {
												 PR_FUNCTION();
  MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
  MFI->setFrameAddressIsTaken(true);

  EVT VT = Op.getValueType();
  DebugLoc dl = Op.getDebugLoc();  // FIXME probably not meaningful
  unsigned Depth = cast<ConstantSDNode>(Op.getOperand(0))->getZExtValue();
  SDValue FrameAddr = DAG.getCopyFromReg(DAG.getEntryNode(), dl,
                                         Picoblaze::BP, VT);
  while (Depth--)
    FrameAddr = DAG.getLoad(VT, dl, DAG.getEntryNode(), FrameAddr,
                            MachinePointerInfo(),
                            false, false, false, 0);
  return FrameAddr;
}

/// getPostIndexedAddressParts - returns true by value, base pointer and
/// offset pointer and addressing mode by reference if this node can be
/// combined with a load / store to form a post-indexed load / store.
bool PicoblazeTargetLowering::getPostIndexedAddressParts(SDNode *N, SDNode *Op,
                                                      SDValue &Base,
                                                      SDValue &Offset,
                                                      ISD::MemIndexedMode &AM,
                                                      SelectionDAG &DAG) const 
{
	PR_FUNCTION();

  LoadSDNode *LD = cast<LoadSDNode>(N);
  if (LD->getExtensionType() != ISD::NON_EXTLOAD)
    return false;

  EVT VT = LD->getMemoryVT();
  if (VT != MVT::i8 && VT != MVT::i16)
    return false;

  if (Op->getOpcode() != ISD::ADD)
    return false;

  if (ConstantSDNode *RHS = dyn_cast<ConstantSDNode>(Op->getOperand(1))) {
    uint64_t RHSC = RHS->getZExtValue();
    if ((VT == MVT::i16 && RHSC != 2) ||
        (VT == MVT::i8 && RHSC != 1))
      return false;

    Base = Op->getOperand(0);
    Offset = DAG.getConstant(RHSC, VT);
    AM = ISD::POST_INC;
    return true;
  }

  return false;
}


const char *PicoblazeTargetLowering::getTargetNodeName(unsigned Opcode) const {
	PR_FUNCTION();
  switch (Opcode) {
  default: return NULL;
  case PicoblazeISD::RET_FLAG:           return "PicoblazeISD::RET_FLAG";
  case PicoblazeISD::RETI_FLAG:          return "PicoblazeISD::RETI_FLAG";
  case PicoblazeISD::RRA:                return "PicoblazeISD::RRA";
  case PicoblazeISD::RLA:                return "PicoblazeISD::RLA";
  case PicoblazeISD::RRC:                return "PicoblazeISD::RRC";
  case PicoblazeISD::CALL:               return "PicoblazeISD::CALL";
  case PicoblazeISD::Wrapper:            return "PicoblazeISD::Wrapper";
  case PicoblazeISD::BR_CC:              return "PicoblazeISD::BR_CC";
  case PicoblazeISD::CMP:                return "PicoblazeISD::CMP";
  case PicoblazeISD::SELECT_CC:          return "PicoblazeISD::SELECT_CC";
  case PicoblazeISD::SHL:                return "PicoblazeISD::SHL";
  case PicoblazeISD::SRA:                return "PicoblazeISD::SRA";
  case PicoblazeISD::PBP:                return "PicoblazeISD::PBP";
  case PicoblazeISD::GETBP:             return "PicoblazeISD::GETBP";
  }

}

bool PicoblazeTargetLowering::isTruncateFree(Type *Ty1,
                                          Type *Ty2) const {
											  PR_FUNCTION();
  if (!Ty1->isIntegerTy() || !Ty2->isIntegerTy())
    return false;

  return (Ty1->getPrimitiveSizeInBits() > Ty2->getPrimitiveSizeInBits());
}

bool PicoblazeTargetLowering::isTruncateFree(EVT VT1, EVT VT2) const {
	PR_FUNCTION();
  if (!VT1.isInteger() || !VT2.isInteger())
    return false;

  return (VT1.getSizeInBits() > VT2.getSizeInBits());
}

bool PicoblazeTargetLowering::isZExtFree(Type *Ty1, Type *Ty2) const {
	PR_FUNCTION();
  // Picoblaze implicitly zero-extends 8-bit results in 16-bit registers.
  return 0 && Ty1->isIntegerTy(8) && Ty2->isIntegerTy(16);
}

bool PicoblazeTargetLowering::isZExtFree(EVT VT1, EVT VT2) const {
	PR_FUNCTION();
  // Picoblaze implicitly zero-extends 8-bit results in 16-bit registers.
  return 0 && VT1 == MVT::i8 && VT2 == MVT::i16;
}

//===----------------------------------------------------------------------===//
//  Other Lowering Code
//===----------------------------------------------------------------------===//

MachineBasicBlock*
PicoblazeTargetLowering::EmitShiftInstr(MachineInstr *MI,
                                     MachineBasicBlock *BB) const {
										 PR_FUNCTION();
  MachineFunction *F = BB->getParent();
  MachineRegisterInfo &RI = F->getRegInfo();
  DebugLoc dl = MI->getDebugLoc();
  const TargetInstrInfo &TII = *getTargetMachine().getInstrInfo();
	return NULL;
}

MachineBasicBlock*
PicoblazeTargetLowering::EmitInstrWithCustomInserter(MachineInstr *MI,
                                                  MachineBasicBlock *BB) const 
{
	PR_FUNCTION();
													  return NULL;
}
  /// ReplaceNodeResults - This callback is invoked when a node result type is
  /// illegal for the target, and the operation was registered to use 'custom'
  /// lowering for that result type.  The target places new result values for
  /// the node in Results (their number and types must exactly match those of
  /// the original return values of the node), or leaves Results empty, which
  /// indicates that the node is not to be custom lowered after all.
  ///
  /// If the target has no operations that require custom lowering, it need not
  /// implement this.  The default implementation aborts.
 void PicoblazeTargetLowering::ReplaceNodeResults(SDNode * N,
                                  SmallVectorImpl<SDValue> &Results,
                                  SelectionDAG & DAG) const {
									  PR_FUNCTION();
    DebugLoc dl = N->getDebugLoc();
	if( N->getOpcode()==ISD::GlobalAddress)
	{
		GlobalAddressSDNode *gn=cast<GlobalAddressSDNode>(N);
	  const GlobalValue *GV = gn->getGlobal();
	  int64_t Offset = gn->getOffset();

	  // Create the TargetGlobalAddress node, folding in the constant offset.
	  SDValue Result = DAG.getConstant(Offset,MVT::i16);
	  Results.push_back(Result);
	}
	
  }

  bool PicoblazeTargetLowering::isLegalAddressingMode(const AddrMode &AM, Type *Ty) const
  {
	    // Allows a sign-extended 16-bit immediate field.
  if (AM.BaseOffs <= -(1LL << 16) || AM.BaseOffs >= (1LL << 16)-1)
    return false;

  // No global is ever allowed as a base.
  if (AM.BaseGV)
    return false;
  if( AM.HasBaseReg)
	 return false;

  return true;
  }
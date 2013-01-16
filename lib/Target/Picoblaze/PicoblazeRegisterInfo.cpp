//===-- PicoblazeRegisterInfo.cpp - Picoblaze Register Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Picoblaze implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Picoblaze-reg-info"

#include "PicoblazeRegisterInfo.h"
#include "Picoblaze.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "PicoblazeMachineFunctionInfo.h"
#include "PicoblazeTargetMachine.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_REGINFO_TARGET_DESC
#include "PicoblazeGenRegisterInfo.inc"

using namespace llvm;

// FIXME: Provide proper call frame setup / destroy opcodes.
PicoblazeRegisterInfo::PicoblazeRegisterInfo(PicoblazeTargetMachine &tm,
                                       const TargetInstrInfo &tii)
  : PicoblazeGenRegisterInfo(Picoblaze::NUM_TARGET_REGS), TM(tm), TII(tii) {
	  PR_FUNCTION();
  StackAlign = TM.getFrameLowering()->getStackAlignment();
  
}

const uint16_t*
PicoblazeRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const 
{
	PR_FUNCTION();
  const TargetFrameLowering *TFI = MF->getTarget().getFrameLowering();
  const Function* F = MF->getFunction();
  static const uint16_t CalleeSavedRegs[] = {
    Picoblaze::BP, 
    0
  };
  static const uint16_t CalleeSavedRegsFP[] = {
    0
  };

  if (TFI->hasFP(*MF))
    return  CalleeSavedRegsFP;
  else
    return  CalleeSavedRegs;

}

BitVector PicoblazeRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
	PR_FUNCTION();
  BitVector Reserved(getNumRegs());
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  // Mark 4 special registers with subregisters as reserved.
  Reserved.set(Picoblaze::BP);
  Reserved.set(Picoblaze::SP);
  Reserved.set(Picoblaze::STATUS);


  return Reserved;
}

const TargetRegisterClass *
PicoblazeRegisterInfo::getPointerRegClass(const MachineFunction &MF, unsigned Kind)
                                                                         const 
{
  PR_FUNCTION();
  return &Picoblaze::GR8RegClass;
}

void PicoblazeRegisterInfo::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const 
{
	PR_FUNCTION();
	MBB.erase(I);
}

void
PicoblazeRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                        int SPAdj, RegScavenger *RS) const 
{
	PR_FUNCTION();
  assert(SPAdj == 0 && "Unexpected");
  unsigned i = 0;
  MachineInstr &MI = *II;
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo *MFI = MF.getFrameInfo();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();  
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();
  DebugLoc dl = MI.getDebugLoc();
  while (!MI.getOperand(i).isFI()) {
    ++i;
    assert(i < MI.getNumOperands() && "Instr doesn't have FrameIndex operand!");
  }

  int FrameIndex = MI.getOperand(i).getIndex();

  unsigned BasePtr = (TFI->hasFP(MF) ? Picoblaze::BP : Picoblaze::SP);
  int Offset = MF.getFrameInfo()->getObjectOffset(FrameIndex);

  // Skip the saved PC
  

  if (!TFI->hasFP(MF))
    Offset += MF.getFrameInfo()->getStackSize();
  else
    Offset += 1; // Skip the saved FPW
  // Fold imm into offset
  //Offset += MI.getOperand(i+1).getImm();

 // unsigned ScratchReg =   RegInfo.createVirtualRegister(&Picoblaze::GR8RegClass);

//  BuildMI(MBB, II, dl, TII.get(Picoblaze::LOAD_REG), ScratchReg)
//		 .addReg(Picoblaze::BP);
		 
      
  //MachineInstrBuilder MIB=
//	  BuildMI(MBB, II, dl, TII.get(Picoblaze::ADD8ri), ScratchReg)
//	     .addReg(ScratchReg)
 //           .addImm(Offset);

 
  //MI.getOperand(i).ChangeToRegister(ScratchReg, false,false,true);
  BuildMI(MBB, II, dl, TII.get(Picoblaze::ADD8ri), Picoblaze::BP)
		 .addReg(Picoblaze::BP)
		 .addImm(Offset);
   II++;
  BuildMI(MBB,II, dl, TII.get(Picoblaze::ADD8ri), Picoblaze::BP)
		 .addReg(Picoblaze::BP)
		 .addImm(-Offset);
  MI.getOperand(i).ChangeToRegister(Picoblaze::BP, false,false,true);
  //MI.getOperand(i).ChangeToImmediate(Offset);
  
}

unsigned PicoblazeRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
	PR_FUNCTION();
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  return TFI->hasFP(MF) ? Picoblaze::BP : Picoblaze::SP;
}

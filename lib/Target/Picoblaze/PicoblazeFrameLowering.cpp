//===-- PicoblazeFrameLowering.cpp - Picoblaze Frame Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Picoblaze implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "PicoblazeFrameLowering.h"
#include "PicoblazeInstrInfo.h"
#include "PicoblazeMachineFunctionInfo.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/DataLayout.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

bool PicoblazeFrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
 PR_FUNCTION();
 
  return (MF.getTarget().Options.DisableFramePointerElim(MF) ||
          MF.getFrameInfo()->hasVarSizedObjects() ||
          MFI->isFrameAddressTaken());
}

bool PicoblazeFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
	PR_FUNCTION();
  return !MF.getFrameInfo()->hasVarSizedObjects();
}

void PicoblazeFrameLowering::emitPrologue(MachineFunction &MF) const 
{
	PR_FUNCTION();

  MachineBasicBlock &MBB = MF.front();   // Prolog goes in entry BB
  MachineFrameInfo *MFI = MF.getFrameInfo();
  PicoblazeMachineFunctionInfo *PicoblazeFI = MF.getInfo<PicoblazeMachineFunctionInfo>();
  const PicoblazeInstrInfo &TII =
    *static_cast<const PicoblazeInstrInfo*>(MF.getTarget().getInstrInfo());

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  // Get the number of bytes to allocate from the FrameInfo.
  uint64_t StackSize = MFI->getStackSize();

  uint64_t NumBytes = 0;
  if (hasFP(MF)) {
    // Calculate required stack adjustment
    uint64_t FrameSize = StackSize - 2;
    NumBytes = FrameSize - PicoblazeFI->getCalleeSavedFrameSize();

    // Get the offset of the stack slot for the EBP register... which is
    // guaranteed to be the last slot by processFunctionBeforeFrameFinalized.
    // Update the frame offset adjustment.
    MFI->setOffsetAdjustment(-NumBytes);

    // Save BP into the appropriate stack slot...
    BuildMI(MBB, MBBI, DL, TII.get(Picoblaze::LOAD_REG))
        .addReg(Picoblaze::BP)
        .addReg(Picoblaze::SP);

    // Update FPW with the new base value...
//    BuildMI(MBB, MBBI, DL, TII.get(Picoblaze::MOV16rr), Picoblaze::FPW)
 //     .addReg(Picoblaze::SPW);

    // Mark the FramePtr as live-in in every block except the entry.
    for (MachineFunction::iterator I = llvm::next(MF.begin()), E = MF.end();
         I != E; ++I)
      I->addLiveIn(Picoblaze::BP);

  } else
    NumBytes = StackSize - PicoblazeFI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
 // while (MBBI != MBB.end() && (MBBI->getOpcode() == Picoblaze::PUSH16r))
 //   ++MBBI;

  if (MBBI != MBB.end())
    DL = MBBI->getDebugLoc();

  if (NumBytes) { // adjust stack pointer: SPW -= numbytes
    // If there is an SUB16ri of SPW immediately before this instruction, merge
    // the two.
    //NumBytes -= mergeSPUpdates(MBB, MBBI, true);
    // If there is an ADD16ri or SUB16ri of SPW immediately after this
    // instruction, merge the two instructions.
    // mergeSPUpdatesDown(MBB, MBBI, &NumBytes);

    if (NumBytes) {
      //MachineInstr *MI =
      //  BuildMI(MBB, MBBI, DL, TII.get(Picoblaze::SUB16ri), Picoblaze::SPW)
      //  .addReg(Picoblaze::SPW).addImm(NumBytes);
      // The SRW implicit def is dead.
      // MI->getOperand(3).setIsDead();
    }
  }

}

void PicoblazeFrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const 
{
	PR_FUNCTION();
	/*
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  PicoblazeMachineFunctionInfo *PicoblazeFI = MF.getInfo<PicoblazeMachineFunctionInfo>();
  const PicoblazeInstrInfo &TII =
    *static_cast<const PicoblazeInstrInfo*>(MF.getTarget().getInstrInfo());

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  unsigned RetOpcode = MBBI->getOpcode();
  DebugLoc DL = MBBI->getDebugLoc();

  switch (RetOpcode) {
  case Picoblaze::RET:
  case Picoblaze::RETI: break;  // These are ok
  default:
    llvm_unreachable("Can only insert epilog into returning blocks");
  }

  // Get the number of bytes to allocate from the FrameInfo
  uint64_t StackSize = MFI->getStackSize();
  unsigned CSSize = PicoblazeFI->getCalleeSavedFrameSize();
  uint64_t NumBytes = 0;

  if (hasFP(MF)) {
    // Calculate required stack adjustment
    uint64_t FrameSize = StackSize - 2;
    NumBytes = FrameSize - CSSize;

    // pop FPW.
    BuildMI(MBB, MBBI, DL, TII.get(Picoblaze::POP16r), Picoblaze::FPW);
  } else
    NumBytes = StackSize - CSSize;

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = prior(MBBI);
    unsigned Opc = PI->getOpcode();
    if (Opc != Picoblaze::POP16r && !PI->isTerminator())
      break;
    --MBBI;
  }

  DL = MBBI->getDebugLoc();

  // If there is an ADD16ri or SUB16ri of SPW immediately before this
  // instruction, merge the two instructions.
  //if (NumBytes || MFI->hasVarSizedObjects())
  //  mergeSPUpdatesUp(MBB, MBBI, StackPtr, &NumBytes);

  if (MFI->hasVarSizedObjects()) {
    BuildMI(MBB, MBBI, DL,
            TII.get(Picoblaze::MOV16rr), Picoblaze::SPW).addReg(Picoblaze::FPW);
    if (CSSize) {
      MachineInstr *MI =
        BuildMI(MBB, MBBI, DL,
                TII.get(Picoblaze::SUB16ri), Picoblaze::SPW)
        .addReg(Picoblaze::SPW).addImm(CSSize);
      // The SRW implicit def is dead.
      MI->getOperand(3).setIsDead();
    }
  } else {
    // adjust stack pointer back: SPW += numbytes
    if (NumBytes) {
      MachineInstr *MI =
        BuildMI(MBB, MBBI, DL, TII.get(Picoblaze::ADD16ri), Picoblaze::SPW)
        .addReg(Picoblaze::SPW).addImm(NumBytes);
      // The SRW implicit def is dead.
      MI->getOperand(3).setIsDead();
    }
  }
  */
}

// FIXME: Can we eleminate these in favour of generic code?
bool
PicoblazeFrameLowering::spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        const TargetRegisterInfo *TRI) const 
{
	PR_FUNCTION();
	return false;
	/*
  if (CSI.empty())
    return false;

  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();

  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();
  PicoblazeMachineFunctionInfo *MFI = MF.getInfo<PicoblazeMachineFunctionInfo>();
  MFI->setCalleeSavedFrameSize(CSI.size() * 2);

  for (unsigned i = CSI.size(); i != 0; --i) {
    unsigned Reg = CSI[i-1].getReg();
    // Add the callee-saved register as live-in. It's killed at the spill.
    MBB.addLiveIn(Reg);
    BuildMI(MBB, MI, DL, TII.get(Picoblaze::PUSH16r))
      .addReg(Reg, RegState::Kill);
  }
  return true;
  */
}

bool
PicoblazeFrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                                 MachineBasicBlock::iterator MI,
                                        const std::vector<CalleeSavedInfo> &CSI,
                                        const TargetRegisterInfo *TRI) const 
{
	PR_FUNCTION();
	return false;
	/*
  if (CSI.empty())
    return false;

  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();

  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();

  for (unsigned i = 0, e = CSI.size(); i != e; ++i)
    BuildMI(MBB, MI, DL, TII.get(Picoblaze::POP16r), CSI[i].getReg());

  return true;
  */
}

void
PicoblazeFrameLowering::processFunctionBeforeFrameFinalized(MachineFunction &MF)
                                                                         const {
	PR_FUNCTION();
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  // Create a frame entry for the FPW register that must be saved.
  if (TFI->hasFP(MF)) {
    int FrameIdx = MF.getFrameInfo()->CreateFixedObject(2, -4, true);
    (void)FrameIdx;
    assert(FrameIdx == MF.getFrameInfo()->getObjectIndexBegin() &&
           "Slot for FPW register must be last in order to be found!");
  }
}

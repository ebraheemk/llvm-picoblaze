//==- PicoblazeFrameLowering.h - Define frame lowering for Picoblaze --*- C++ -*--==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef Picoblaze_FRAMEINFO_H
#define Picoblaze_FRAMEINFO_H

#include "Picoblaze.h"
#include "PicoblazeSubtarget.h"
#include "llvm/Target/TargetFrameLowering.h"

namespace llvm {
  class PicoblazeSubtarget;

class PicoblazeFrameLowering : public TargetFrameLowering {
protected:
  const PicoblazeSubtarget &STI;

public:
  explicit PicoblazeFrameLowering(const PicoblazeSubtarget &sti)
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 2, -2), STI(sti) {
  }

  /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
  /// the function.
  virtual void emitPrologue(MachineFunction &MF) const;
  virtual void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const;

  virtual bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MI,
                                 const std::vector<CalleeSavedInfo> &CSI,
                                 const TargetRegisterInfo *TRI) const;
  virtual bool restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MI,
                                   const std::vector<CalleeSavedInfo> &CSI,
                                   const TargetRegisterInfo *TRI) const;

  bool hasFP(const MachineFunction &MF) const;
  bool hasReservedCallFrame(const MachineFunction &MF) const;
  void processFunctionBeforeFrameFinalized(MachineFunction &MF) const;
};

} // End llvm namespace

#endif

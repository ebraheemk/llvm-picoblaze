//===-- PicoblazeTargetMachine.h - Define TargetMachine for Picoblaze -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Picoblaze specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//


#ifndef LLVM_TARGET_Picoblaze_TARGETMACHINE_H
#define LLVM_TARGET_Picoblaze_TARGETMACHINE_H

#include "PicoblazeInstrInfo.h"
#include "PicoblazeISelLowering.h"
#include "PicoblazeFrameLowering.h"
#include "PicoblazeSelectionDAGInfo.h"
#include "PicoblazeRegisterInfo.h"
#include "PicoblazeSubtarget.h"
#include "llvm/DataLayout.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetTransformImpl.h"

namespace llvm {

/// PicoblazeTargetMachine
///
class PicoblazeTargetMachine : public LLVMTargetMachine {
  PicoblazeSubtarget        Subtarget;
  const DataLayout       DL;       // Calculates type size & alignment
  PicoblazeInstrInfo        InstrInfo;
  PicoblazeTargetLowering   TLInfo;
  PicoblazeSelectionDAGInfo TSInfo;
  PicoblazeFrameLowering    FrameLowering;
  ScalarTargetTransformImpl STTI;
  VectorTargetTransformImpl VTTI;

public:
  PicoblazeTargetMachine(const Target &T, StringRef TT,
                      StringRef CPU, StringRef FS, const TargetOptions &Options,
                      Reloc::Model RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);

  virtual const TargetFrameLowering *getFrameLowering() const {
    return &FrameLowering;
  }
  virtual const PicoblazeInstrInfo *getInstrInfo() const  { return &InstrInfo; }
  virtual const DataLayout *getDataLayout() const     { return &DL;}
  virtual const PicoblazeSubtarget *getSubtargetImpl() const { return &Subtarget; }

  virtual const TargetRegisterInfo *getRegisterInfo() const {
    return &InstrInfo.getRegisterInfo();
  }

  virtual const PicoblazeTargetLowering *getTargetLowering() const {
    return &TLInfo;
  }

  virtual const PicoblazeSelectionDAGInfo* getSelectionDAGInfo() const {
    return &TSInfo;
  }
  virtual const ScalarTargetTransformInfo *getScalarTargetTransformInfo()const {
    return &STTI;
  }
  virtual const VectorTargetTransformInfo *getVectorTargetTransformInfo()const {
    return &VTTI;
  }
  virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);
}; // PicoblazeTargetMachine.

} // end namespace llvm

#endif // LLVM_TARGET_Picoblaze_TARGETMACHINE_H

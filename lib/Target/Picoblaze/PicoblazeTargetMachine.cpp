//===-- PicoblazeTargetMachine.cpp - Define TargetMachine for Picoblaze ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Top-level implementation for the Picoblaze target.
//
//===----------------------------------------------------------------------===//

#include "PicoblazeTargetMachine.h"
#include "Picoblaze.h"
#include "llvm/PassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

extern "C" void LLVMInitializePicoblazeTarget() {
  // Register the target.
  RegisterTargetMachine<PicoblazeTargetMachine> X(ThePicoblazeTarget);
}

PicoblazeTargetMachine::PicoblazeTargetMachine(const Target &T,
                                         StringRef TT,
                                         StringRef CPU,
                                         StringRef FS,
                                         const TargetOptions &Options,
                                         Reloc::Model RM, CodeModel::Model CM,
                                         CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL),
    Subtarget(TT, CPU, FS),
    // FIXME: Check DataLayout string.
    DL("e-p:8:8:8-i1:8:8-i8:8:8-i16:8:8-i32:8:8-n8:8"),
    InstrInfo(*this), TLInfo(*this), TSInfo(*this),
    FrameLowering(Subtarget), STTI(&TLInfo), VTTI(&TLInfo) { 
		PR_FUNCTION();
}

namespace {
/// Picoblaze Code Generator Pass Configuration Options.
class PicoblazePassConfig : public TargetPassConfig {
public:
  PicoblazePassConfig(PicoblazeTargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  PicoblazeTargetMachine &getPicoblazeTargetMachine() const {
    return getTM<PicoblazeTargetMachine>();
  }

  virtual bool addInstSelector();
  virtual bool addPreEmitPass();
};
} // namespace

TargetPassConfig *PicoblazeTargetMachine::createPassConfig(PassManagerBase &PM) {
	PR_FUNCTION();
  return new PicoblazePassConfig(this, PM);
}

bool PicoblazePassConfig::addInstSelector() {
	PR_FUNCTION();
  // Install an instruction selector.
  addPass(createPicoblazeISelDag(getPicoblazeTargetMachine(), getOptLevel()));
  return false;
}

bool PicoblazePassConfig::addPreEmitPass() {
	PR_FUNCTION();
  // Must run branch selection immediately preceding the asm printer.
  addPass(createPicoblazeBranchSelectionPass());
  return false;
}

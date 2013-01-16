//===-- PicoblazeBranchSelector.cpp - Emit long conditional branches ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that scans a machine function to determine which
// conditional branches need more than 10 bits of displacement to reach their
// target basic block.  It does this in two passes; a calculation of basic block
// positions pass, and a branch pseudo op to machine branch opcode pass.  This
// pass should be run last, just before the assembly printer.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "Picoblaze-branch-select"
#include "Picoblaze.h"
#include "PicoblazeInstrInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/MathExtras.h"
using namespace llvm;

STATISTIC(NumExpanded, "Number of branches expanded to long format");

namespace {
  struct PicoblazeBSel : public MachineFunctionPass {
    static char ID;
    PicoblazeBSel() : MachineFunctionPass(ID) {}

    /// BlockSizes - The sizes of the basic blocks in the function.
    std::vector<unsigned> BlockSizes;

    virtual bool runOnMachineFunction(MachineFunction &Fn);

    virtual const char *getPassName() const {
      return "Picoblaze Branch Selector";
    }
  };
  char PicoblazeBSel::ID = 0;
}

/// createPicoblazeBranchSelectionPass - returns an instance of the Branch
/// Selection Pass
///
FunctionPass *llvm::createPicoblazeBranchSelectionPass() {
	PR_FUNCTION();
  return new PicoblazeBSel();
}

bool PicoblazeBSel::runOnMachineFunction(MachineFunction &Fn)
{
  PR_FUNCTION();
  return true;
}

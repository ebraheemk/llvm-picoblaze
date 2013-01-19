//===-- MachineFunctionPass.cpp -------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the definitions of the MachineFunctionPass members.
//
//===----------------------------------------------------------------------===//

#include "llvm/Function.h"
#include <sstream>
#include "llvm/Support/raw_ostream.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/MachineFunctionAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunction.h"
using namespace llvm;

Pass *MachineFunctionPass::createPrinterPass(raw_ostream &O,
                                             const std::string &Banner) const {
  return createMachineFunctionPrinterPass(O, Banner);
}
#ifndef NDEBUG
 // {
	  static int passnum=0;
 // }
#endif
bool MachineFunctionPass::runOnFunction(Function &F) {
  // Do not codegen any 'available_externally' functions at all, they have
  // definitions outside the translation unit.
  if (F.hasAvailableExternallyLinkage())
    return false;

  MachineFunction &MF = getAnalysis<MachineFunctionAnalysis>().getMF();

  bool bret=runOnMachineFunction(MF);
#ifndef NDEBUG
  {
	std::string ErrorInfo;
	char  fname[16];
	sprintf_s(fname,16,"pass%02d.txt",passnum);
	passnum++;
	raw_fd_ostream fd(fname,ErrorInfo);

	fd<<this->getPassName() <<'\n';
	MF.print(fd);
	

  }
#endif
  return bret;
}

void MachineFunctionPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<MachineFunctionAnalysis>();
  AU.addPreserved<MachineFunctionAnalysis>();

  // MachineFunctionPass preserves all LLVM IR passes, but there's no
  // high-level way to express this. Instead, just list a bunch of
  // passes explicitly. This does not include setPreservesCFG,
  // because CodeGen overloads that to mean preserving the MachineBasicBlock
  // CFG in addition to the LLVM IR CFG.
  AU.addPreserved<AliasAnalysis>();
  AU.addPreserved("scalar-evolution");
  AU.addPreserved("iv-users");
  AU.addPreserved("memdep");
  AU.addPreserved("live-values");
  AU.addPreserved("domtree");
  AU.addPreserved("domfrontier");
  AU.addPreserved("loops");
  AU.addPreserved("lda");

  FunctionPass::getAnalysisUsage(AU);
}

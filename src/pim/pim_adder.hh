#ifndef __PIM_ADDER__
#define __PIM_ADDER__

#include "params/PIMAdder.hh"
#include "pim_kernel.hh"

class PIMAdder : public PIMKernel {
public:
  typedef PIMAdderParams Params;
  PIMAdder(const Params *p);
  virtual dataType doCompute() override;
};

#endif

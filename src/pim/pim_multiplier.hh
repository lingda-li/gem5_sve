#ifndef __PIM_ADDER__
#define __PIM_ADDER__

#include "params/PIMMultiplier.hh"
#include "pim_kernel.hh"

class PIMMultiplier : public PIMKernel {
public:
  typedef PIMMultiplierParams Params;
  PIMMultiplier(const Params *p);
  virtual dataType doCompute() override;
};

#endif

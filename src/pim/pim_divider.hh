#ifndef __PIM_ADDER__
#define __PIM_ADDER__

#include "params/PIMDivider.hh"
#include "pim/pim_kernel.hh"

class PIMDivider : public PIMKernel {
public:
  typedef PIMDividerParams Params;
  PIMDivider(const Params *p);
  virtual dataType doCompute() override;
};

#endif

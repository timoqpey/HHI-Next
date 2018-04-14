
#pragma once


#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"
#include "Quant.h"
#include "QuantRDOQ.h"
#include "QuantRDOQ2.h"





class TCQ : public QuantRDOQ2
{
public:
  TCQ( const Quant* other );
  virtual ~TCQ();

  virtual void init( UInt uiMaxTrSize,
                     Bool useRDOQ = false,
                     Bool useRDOQTS = false,
                     UInt uiAltResiCompId = 0,
#if T0196_SELECTIVE_RDOQ
                     Bool useSelectiveRDOQ = false
#endif
                   );

  virtual void quant  ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );
  virtual void dequant( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP );

private:
  void* p;
};




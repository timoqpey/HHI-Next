

#include "TCQ.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"

#include <bitset>






namespace TCQIntern
{
  static const int tab_nextState[ 4 ][ 2 ]  = { {0,2}, {2,0}, {1,3}, {3,1} };
  static const int tab_nextQSet [ 4 ]       = {  0,     0,     1,     1    };
  static const int tcq_qp_offset            = -5; //-6;

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   R A T E   E S T I M A T O R                                        =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct CoeffFracBits
  {
    int32_t bits[6];
  };

  class RateEstimator
  {
  public:
    RateEstimator () {}
    void initBlock( const TransformUnit& tu, const ComponentID     compID );
    void initCtx  ( const TransformUnit& tu, const FracBitsAccess& fracBitsAccess );

    __inline  bool                  luma          ()                    const { return m_compID == COMPONENT_Y; }
    __inline  int32_t               widthInSbb    ()                    const { return m_widthInSbb; }
    __inline  int32_t               heightInSbb   ()                    const { return m_heightInSbb; }
    __inline  int32_t               numCoeff      ()                    const { return m_numCoeff; }
    __inline  int32_t               numSbb        ()                    const { return m_numSbb; }
    __inline  int32_t               sbbSize       ()                    const { return m_sbbSize; }
    __inline  int32_t               sbbPos        ( unsigned scanIdx  ) const { return m_scanSbbId2SbbPos [scanIdx>>m_log2SbbSize]; }
    __inline  int32_t               rasterPos     ( unsigned scanIdx  ) const { return m_scanId2BlkPos    [scanIdx]; }
    __inline  int32_t               posX          ( unsigned scanIdx  ) const { return m_scanId2PosX      [scanIdx]; }
    __inline  int32_t               posY          ( unsigned scanIdx  ) const { return m_scanId2PosY      [scanIdx]; }
    __inline  const NbInfoSbb&      nbInfoSbb     ( unsigned scanIdx  ) const { return m_scanId2NbInfoSbb [scanIdx]; }
    __inline  const NbInfoOut*      nbInfoOut     ()                    const { return m_scanId2NbInfoOut; }
    __inline  const BinFracBits*    sigSbbFracBits()                    const { return m_sigSbbFracBits; }
    __inline  const BinFracBits*    sigFlagBits   ( unsigned stateId  ) const { return m_sigFracBits[stateId>>1]; }
    __inline  const CoeffFracBits*  gtxFracBits   ( unsigned stateId  ) const { return m_gtxFracBits[stateId>>1]; }
    __inline  int32_t               lastOffset    ( unsigned scanIdx )  const { return m_lastBitsX[ m_scanId2PosX[scanIdx] ] + m_lastBitsY[ m_scanId2PosY[scanIdx] ]; }

  private:
    void  xSetLastCoeffOffset ( const FracBitsAccess& fracBitsAccess, const TransformUnit& tu );
    void  xSetSigSbbFracBits  ( const FracBitsAccess& fracBitsAccess );
    void  xSetSigFlagBits     ( const FracBitsAccess& fracBitsAccess );
    void  xSetGtxFlagBits     ( const FracBitsAccess& fracBitsAccess );

  private:
    static const unsigned sm_numCtxSetsSig    = 2;
    static const unsigned sm_numCtxSetsGtx    = 2;
    static const unsigned sm_maxNumSigSbbCtx  = 2;
    static const unsigned sm_maxNumSigCtx     = 18;
    static const unsigned sm_maxNumGtxCtx     = 21;

  private:
    ComponentID       m_compID;
    ChannelType       m_chType;
    unsigned          m_width;
    unsigned          m_height;
    bool              m_1dBlk;
    unsigned          m_numCoeff;
    unsigned          m_numSbb;
    unsigned          m_log2SbbWidth;
    unsigned          m_log2SbbHeight;
    unsigned          m_log2SbbSize;
    unsigned          m_sbbSize;
    unsigned          m_sbbMask;
    unsigned          m_widthInSbb;
    unsigned          m_heightInSbb;
    CoeffScanType     m_scanType;
    const unsigned*   m_scanSbbId2SbbPos;
    const unsigned*   m_scanId2BlkPos;
    const unsigned*   m_scanId2PosX;
    const unsigned*   m_scanId2PosY;
    const NbInfoSbb*  m_scanId2NbInfoSbb;
    const NbInfoOut*  m_scanId2NbInfoOut;
    int32_t           m_lastBitsX      [ MAX_TU_SIZE ];
    int32_t           m_lastBitsY      [ MAX_TU_SIZE ];
    BinFracBits       m_sigSbbFracBits [ sm_maxNumSigSbbCtx ];
    BinFracBits       m_sigFracBits    [ sm_numCtxSetsSig   ][ sm_maxNumSigCtx ];
    CoeffFracBits     m_gtxFracBits    [ sm_numCtxSetsGtx   ][ sm_maxNumGtxCtx ];
  };

  void RateEstimator::initBlock( const TransformUnit& tu, const ComponentID compID )
  {
    CHECKD( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag(), "ext precision is not supported" );

    const CompArea& area  = tu.blocks[ compID ];
    m_compID              = compID;
    m_chType              = toChannelType( m_compID );
    m_width               = area.width;
    m_height              = area.height;
    m_1dBlk               = ( m_width == 1 || m_height == 1 );
    m_numCoeff            = m_width * m_height;
    const bool      no4x4 = ( ( m_width & 3 ) != 0 || ( m_height & 3 ) != 0 );
    m_log2SbbWidth        = ( !m_1dBlk ? ( no4x4 ? 1 : 2 ) : ( m_width  == 1 ? 0 : ( m_width  & 3 ) ? 1 : 2 ) );
    m_log2SbbHeight       = ( !m_1dBlk ? ( no4x4 ? 1 : 2 ) : ( m_height == 1 ? 0 : ( m_height & 3 ) ? 1 : 2 ) );
    m_log2SbbSize         = m_log2SbbWidth + m_log2SbbHeight;
    m_sbbSize             = ( 1 << m_log2SbbSize );
    m_sbbMask             = m_sbbSize - 1;
    m_widthInSbb          = m_width  >> m_log2SbbWidth;
    m_heightInSbb         = m_height >> m_log2SbbHeight;
    m_numSbb              = m_widthInSbb * m_heightInSbb;
    m_scanType            = CoeffScanType( TU::getCoefScanIdx( tu, m_compID ) );
    SizeType        hsbb  = gp_sizeIdxInfo->idxFrom( m_widthInSbb  );
    SizeType        vsbb  = gp_sizeIdxInfo->idxFrom( m_heightInSbb );
    SizeType        hsId  = gp_sizeIdxInfo->idxFrom( m_width  );
    SizeType        vsId  = gp_sizeIdxInfo->idxFrom( m_height );
    m_scanSbbId2SbbPos    = g_scanOrder     [ SCAN_UNGROUPED   ][ m_scanType ][ hsbb ][ vsbb ];
    m_scanId2BlkPos       = g_scanOrder     [ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ];
    m_scanId2PosX         = g_scanOrderPosXY[ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ][ 0 ];
    m_scanId2PosY         = g_scanOrderPosXY[ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ][ 1 ];
    m_scanId2NbInfoSbb    = g_scanId2NbInfoSbb                  [ m_scanType ][ hsId ][ vsId ];
    m_scanId2NbInfoOut    = g_scanId2NbInfoOut                  [ m_scanType ][ hsId ][ vsId ];
  }

  void RateEstimator::initCtx( const TransformUnit& tu, const FracBitsAccess& fracBitsAccess )
  {
    xSetSigSbbFracBits  ( fracBitsAccess );
    xSetSigFlagBits     ( fracBitsAccess );
    xSetGtxFlagBits     ( fracBitsAccess );
    xSetLastCoeffOffset ( fracBitsAccess, tu );
  }

  void RateEstimator::xSetLastCoeffOffset( const FracBitsAccess& fracBitsAccess, const TransformUnit& tu )
  {
    int32_t cbfDeltaBits = 0;
    if( m_compID == COMPONENT_Y && !CU::isIntra(*tu.cu) && !tu.depth )
    {
      const BinFracBits bits  = fracBitsAccess.getFracBitsArray( Ctx::QtRootCbf() );
      cbfDeltaBits            = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
    }
    else
    {



      bool previousCbf = false;
      if( tu.cu->mode1dPartitions && canUse1dPartitions( m_compID ) )
      {
        previousCbf = TU::getPrevTuCbfAtDepth( tu, tu.depth, m_compID );
      }


      BinFracBits bits = fracBitsAccess.getFracBitsArray( Ctx::QtCbf[m_chType << ( tu.cs->sps->getSpsNext().getAltResiCompId() == 2 ? 1 : 0 )]( DeriveCtx::CtxQtCbf( m_compID, tu.depth, tu.cu->mode1dPartitions, previousCbf ) ) );

      if( tu.cs->sps->getSpsNext().getAltResiCompId() == 2 && m_compID == COMPONENT_Cr && !( tu.cu->mode1dPartitions && canUse1dPartitions( m_compID ) ) )
      {
        bits = fracBitsAccess.getFracBitsArray( Ctx::QtCbf[2]( tu.cbf[COMPONENT_Cb] ? 3 : 2 ) );
      }

      cbfDeltaBits            = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
    }

    static const unsigned prefixCtx[] = { 0, 0, 0, 3, 6, 10, 15, 21 };
    uint32_t              ctxBits  [ LAST_SIGNIFICANT_GROUPS ];
    for( unsigned xy = 0; xy < 2; xy++ )
    {
      int32_t             bitOffset   = ( xy ? cbfDeltaBits : 0 );
      int32_t*            lastBits    = ( xy ? m_lastBitsY : m_lastBitsX );
      const unsigned      size        = ( xy ? m_height : m_width );
      const unsigned      log2Size    = g_aucNextLog2[ size ];
      const bool          useYCtx     = ( !m_1dBlk && m_scanType == SCAN_VER ? ( xy == 0 ) : ( xy != 0 ) );
      const CtxSet&       ctxSetLast  = ( useYCtx ? Ctx::LastY : Ctx::LastX )[ m_chType ];
      const unsigned      lastShift   = ( m_compID == COMPONENT_Y ? (log2Size+1)>>2 : ( tu.cs->pcv->rectCUs ? Clip3<unsigned>(0,2,size>>3) : log2Size-2 ) );
      const unsigned      lastOffset  = ( m_compID == COMPONENT_Y ? ( tu.cs->pcv->rectCUs ? prefixCtx[log2Size] : 3*(log2Size-2)+((log2Size-1)>>2) ) : 0 );
      uint32_t            sumFBits    = 0;
      unsigned            maxCtxId    = g_uiGroupIdx[ size - 1 ];
      for( unsigned ctxId = 0; ctxId < maxCtxId; ctxId++ )
      {
        const BinFracBits bits  = fracBitsAccess.getFracBitsArray( ctxSetLast( lastOffset + ( ctxId >> lastShift ) ) );
        ctxBits[ ctxId ]        = sumFBits + bits.intBits[0] + ( ctxId>3 ? ((ctxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
        sumFBits               +=            bits.intBits[1];
      }
      ctxBits  [ maxCtxId ]     = sumFBits + ( maxCtxId>3 ? ((maxCtxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
      for( unsigned pos = 0; pos < size; pos++ )
      {
        lastBits[ pos ]         = ctxBits[ g_uiGroupIdx[ pos ] ];
      }
    }
  }

  void RateEstimator::xSetSigSbbFracBits( const FracBitsAccess& fracBitsAccess )
  {
    const CtxSet& ctxSet = Ctx::SigCoeffGroup[ m_chType ];
    for( unsigned ctxId = 0; ctxId < sm_maxNumSigSbbCtx; ctxId++ )
    {
      m_sigSbbFracBits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
    }
  }

  void RateEstimator::xSetSigFlagBits( const FracBitsAccess& fracBitsAccess )
  {
    for( unsigned ctxSetId = 0; ctxSetId < sm_numCtxSetsSig; ctxSetId++ )
    {
      BinFracBits*    bits    = m_sigFracBits [ ctxSetId ];
      const CtxSet&   ctxSet  = Ctx::SigFlag  [ 6 + m_chType + 2*ctxSetId ];
      const unsigned  numCtx  = ( m_compID == COMPONENT_Y ? 18 : 12 );
      for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
      {
        bits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
      }
    }
  }

  void RateEstimator::xSetGtxFlagBits( const FracBitsAccess& fracBitsAccess )
  {
    const CtxSet&   ctxSetGtx   = Ctx::GreaterOneFlag [ 10 + m_chType ];
    const CtxSet&   ctxSetGt1a  = Ctx::GreaterOneFlag [ 12 + m_chType ];
    const CtxSet&   ctxSetGt1b  = Ctx::GreaterOneFlag [ 14 + m_chType ];
    const unsigned  numCtx      = ( m_compID == COMPONENT_Y ? 21 : 11 );
    for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
    {
      BinFracBits    fbGtx  = fracBitsAccess.getFracBitsArray( ctxSetGtx ( ctxId ) );
      BinFracBits    fbGt1a = fracBitsAccess.getFracBitsArray( ctxSetGt1a( ctxId ) );
      BinFracBits    fbGt1b = fracBitsAccess.getFracBitsArray( ctxSetGt1b( ctxId ) );
      CoeffFracBits& cba    = m_gtxFracBits[0][ ctxId ];
      CoeffFracBits& cbb    = m_gtxFracBits[1][ ctxId ];
      cba.bits[0] = 0;
      cba.bits[1] =                   + fbGt1a.intBits[0] + (1<<SCALE_BITS);
      cba.bits[2] = fbGt1a.intBits[1] + fbGtx .intBits[0] + (1<<SCALE_BITS);
      cba.bits[3] = cba.bits[2]       + fbGtx .intBits[1];
      cba.bits[4] = cba.bits[3]       + fbGtx .intBits[1];
      cba.bits[5] = cba.bits[4]       + fbGtx .intBits[1] - (int32_t)fbGtx.intBits[0];
      cbb.bits[0] = 0;
      cbb.bits[1] =                   + fbGt1b.intBits[0] + (1<<SCALE_BITS);
      cbb.bits[2] = fbGt1b.intBits[1] + fbGtx .intBits[0] + (1<<SCALE_BITS);
      cbb.bits[3] = cbb.bits[2]       + fbGtx .intBits[1];
      cbb.bits[4] = cbb.bits[3]       + fbGtx .intBits[1];
      cbb.bits[5] = cbb.bits[4]       + fbGtx .intBits[1] - (int32_t)fbGtx.intBits[0];
    }
  }





  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   D A T A   S T R U C T U R E S                                      =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  enum ScanPosType { SCAN_ISCSBB = 0, SCAN_SOCSBB = 1, SCAN_EOCSBB = 2 };

  struct ScanInfo
  {
    const int     sbbSize;
    const int     numSbb;
    int           scanIdx;
    int           rasterPos;
    int           lastOffset;
    unsigned      sigCtxOffsetNext;
    unsigned      gtxCtxOffsetNext;
    int           insidePos;
    int           nextInsidePos;
    NbInfoSbb     nextNbInfoSbb;
    bool          sosbb;
    bool          eosbb;
    bool          socsbb;
    bool          eocsbb;
    int           sbbPos;
    int           nextSbbRight;
    int           nextSbbBelow;
  protected:
    ScanInfo( int _sbbSize, int _numSbb ) : sbbSize( _sbbSize ), numSbb( _numSbb ) {}
  };

  class ScanData : public ScanInfo
  {
  public:
    ScanData( const RateEstimator& rateEst, int firstPos )
      : ScanInfo          ( rateEst.sbbSize(), rateEst.numSbb() )
      , m_rateEst         ( rateEst )
      , m_luma            ( m_rateEst.luma() )
      , m_sbbMask         ( sbbSize - 1 )
      , m_widthInSbb      ( m_rateEst.widthInSbb() )
      , m_heightInSbb     ( m_rateEst.heightInSbb() )
      , m_numCoeffMinus1  ( m_rateEst.numCoeff() - 1 )
      , m_numCoeffMinusSbb( m_rateEst.numCoeff() - sbbSize )
    {
      xSet( firstPos );
    }
    __inline bool  valid ()         const { return scanIdx >= 0; }
    void           next  ()               { xSet( scanIdx-1 ); }
    void           set   ( int id )       { xSet(id); }

  private:
    __inline void xSet( int _scanIdx )
    {
      scanIdx = _scanIdx;
      if( scanIdx >= 0 )
      {
        rasterPos               = m_rateEst.rasterPos   ( scanIdx );
        sbbPos                  = m_rateEst.sbbPos      ( scanIdx );
        lastOffset              = m_rateEst.lastOffset  ( scanIdx );
        insidePos               = scanIdx & m_sbbMask;
        sosbb                   = ( insidePos == m_sbbMask );
        eosbb                   = ( insidePos == 0 );
        socsbb                  = ( sosbb && scanIdx > sbbSize && scanIdx < m_numCoeffMinus1   );
        eocsbb                  = ( eosbb && scanIdx > 0       && scanIdx < m_numCoeffMinusSbb );
        if( scanIdx )
        {
          const int nextScanIdx = scanIdx - 1;
          const int diag        = m_rateEst.posX( nextScanIdx ) + m_rateEst.posY( nextScanIdx );
          if( m_luma )
          {
            sigCtxOffsetNext    = ( diag < 2 ? 12 : diag < 5 ?  6 : 0 );
            gtxCtxOffsetNext    = ( diag < 1 ? 16 : diag < 3 ? 11 : diag < 10 ? 6 : 1 );
          }
          else
          {
            sigCtxOffsetNext    = ( diag < 2 ? 6 : 0 );
            gtxCtxOffsetNext    = ( diag < 1 ? 6 : 1 );
          }
          nextInsidePos         = nextScanIdx & m_sbbMask;
          nextNbInfoSbb         = m_rateEst.nbInfoSbb( nextScanIdx );
          if( eosbb )
          {
            const int nextSbbPos  = m_rateEst.sbbPos( nextScanIdx );
            const int nextSbbPosY = nextSbbPos               / m_widthInSbb;
            const int nextSbbPosX = nextSbbPos - nextSbbPosY * m_widthInSbb;
            nextSbbRight          = ( nextSbbPosX < m_widthInSbb  - 1 ? nextSbbPos + 1            : 0 );
            nextSbbBelow          = ( nextSbbPosY < m_heightInSbb - 1 ? nextSbbPos + m_widthInSbb : 0 );
          }
        }
      }
    }
  private:
    const RateEstimator& m_rateEst;
    const bool           m_luma;
    const int            m_sbbMask;
    const int            m_widthInSbb;
    const int            m_heightInSbb;
    const int            m_numCoeffMinus1;
    const int            m_numCoeffMinusSbb;
  };


  struct PQData
  {
    TCoeff  absLevel;
    int64_t deltaDist;
  };


  struct Decision
  {
    int64_t rdCost;
    TCoeff  absLevel;
    int     prevId;
  };




  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   P R E - Q U A N T I Z E R                                          =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class Quantizer
  {
  public:
    Quantizer() {}

    void  dequantBlock  ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff,  const bool tcq )   const;
    void  initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, const bool tcq );

    __inline void    preQuantCoeff     ( const TCoeff absCoeff, PQData* pqData )    const;
    __inline TCoeff  getLastThreshold  ()                                           const { return m_thresLast; }
    __inline TCoeff  getSSbbThreshold  ()                                           const { return m_thresSSbb; }
    __inline int64_t getDeltaDistortion( const TCoeff absCoeff, const TCoeff qIdx ) const
    {
      int64_t scaledOrg = int64_t( absCoeff ) * m_QScale;
      int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;
      int64_t deltaDist = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
      return  deltaDist;
    }

  private:
    void  xDequantTCQ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff )   const;
    void  xDequantStd( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff )   const;

  private:
    // quantization
    int               m_QShift;
    int64_t           m_QAdd;
    int64_t           m_QScale;
    TCoeff            m_maxQIdx;
    TCoeff            m_thresLast;
    TCoeff            m_thresSSbb;
    // distortion normalization
    int               m_DistShift;
    int64_t           m_DistAdd;
    int64_t           m_DistStepAdd;
    int64_t           m_DistOrgFact;
  };

  __inline int ceil_log2( uint64_t x )
  {
    static const uint64_t t[6] = { 0xFFFFFFFF00000000ull, 0x00000000FFFF0000ull, 0x000000000000FF00ull, 0x00000000000000F0ull, 0x000000000000000Cull, 0x0000000000000002ull };
    int y = (((x & (x - 1)) == 0) ? 0 : 1);
    int j = 32;
    for( int i = 0; i < 6; i++)
    {
      int k = (((x & t[i]) == 0) ? 0 : j);
      y += k;
      x >>= k;
      j >>= 1;
    }
    return y;
  }

  void Quantizer::initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, const bool tcq )
  {
    CHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );
    CHECKD( lambda <= 0.0, "Lambda must be greater than 0" );

    const int         qp                    = cQP.Qp + ( tcq ? tcq_qp_offset : 0 );
    const int         qpPer                 = ( qp + 36 ) / 6 - 6;
    const int         qpRem                 =   qp - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const CompArea&   area                  = tu.blocks[ compID ];
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );

    // quant parameters
    m_QShift                    = QUANT_SHIFT + qpPer + transformShift;
    m_QAdd                      = -( ( 3 << m_QShift ) >> 1 );
    Intermediate_Int  invShift  = IQUANT_SHIFT - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    m_QScale                    = ( TU::needsBlockSizeTrafoScale( area ) ? ( ( ( g_quantScales[ qpRem ] * TU::getBlockSizeTrafoScaleForQuant( area ) + ( 1 << ( ADJ_QUANT_SHIFT - 1 ) ) ) >> ADJ_QUANT_SHIFT ) ) : g_quantScales[ qpRem ] );
    const unsigned    qIdxBD    = std::min<unsigned>( maxLog2TrDynamicRange + 1, 8*sizeof(Intermediate_Int) + invShift - IQUANT_SHIFT - 1 );
    m_maxQIdx                   = ( 1 << (qIdxBD-1) ) - 4;
    m_thresLast                 = TCoeff( ( int64_t(3) << m_QShift ) / ( 4 * m_QScale ) );
    m_thresSSbb                 = TCoeff( ( int64_t(3) << m_QShift ) / ( 4 * m_QScale ) );

    // distortion calculation parameters
    const int64_t qScale        = g_quantScales[ qpRem ];
    const int     nomDShift     = SCALE_BITS - 2 * ( nomTransformShift + channelBitDepth - 8 ) + m_QShift;
    const double  qScale2       = double( qScale * qScale );
    const double  nomDistFactor = ( nomDShift < 0 ? 1.0/(double(int64_t(1)<<(-nomDShift))*qScale2*lambda) : double(int64_t(1)<<nomDShift)/(qScale2*lambda) );
    const int64_t pow2dfShift   = (int64_t)( nomDistFactor * qScale2 ) + 1;
    const int     dfShift       = ceil_log2( pow2dfShift );
    m_DistShift                 = 62 + m_QShift - 2*maxLog2TrDynamicRange - dfShift;
    m_DistAdd                   = (int64_t(1) << m_DistShift) >> 1;
    m_DistStepAdd               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+m_QShift)) + .5 );
    m_DistOrgFact               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+1       )) + .5 );
  }

  void Quantizer::dequantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff,  const bool tcq ) const
  {
    if( tcq )
    {
      xDequantTCQ( tu, compID, cQP, recCoeff );
    }
    else
    {
      xDequantStd( tu, compID, cQP, recCoeff );
    }
  }

  void Quantizer::xDequantTCQ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff  )  const
  {
    CHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );

    //----- set basic parameters -----
    const CompArea&     area      = tu.blocks[ compID ];
    const int           numCoeff  = area.area();
    const SizeType      hsId      = gp_sizeIdxInfo->idxFrom( area.width  );
    const SizeType      vsId      = gp_sizeIdxInfo->idxFrom( area.height );
    const CoeffScanType scanType  = CoeffScanType( TU::getCoefScanIdx( tu, compID ) );
    const unsigned*     scan      = g_scanOrder[ SCAN_GROUPED_4x4 ][ scanType ][ hsId ][ vsId ];
    const TCoeff*       qCoeff    = tu.getCoeffs( compID ).buf;
          TCoeff*       tCoeff    = recCoeff.buf;

    //----- reset coefficients and get last scan index -----
    ::memset( tCoeff, 0, numCoeff * sizeof(TCoeff) );
    int lastScanIdx = -1;
    for( int scanIdx = numCoeff - 1; scanIdx >= 0; scanIdx-- )
    {
      if( qCoeff[ scan[ scanIdx ] ] )
      {
        lastScanIdx = scanIdx;
        break;
      }
    }
    if( lastScanIdx < 0 )
    {
      return;
    }

    //----- set dequant parameters -----
    const int         qp                    = cQP.Qp + tcq_qp_offset;
    const int         qpPer                 = ( qp + 36 ) / 6 - 6;
    const int         qpRem                 =   qp - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const TCoeff      minTCoeff             = -( 1 << maxLog2TrDynamicRange );
    const TCoeff      maxTCoeff             =  ( 1 << maxLog2TrDynamicRange ) - 1;
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );
    Intermediate_Int  shift     = IQUANT_SHIFT - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    Intermediate_Int  invQScale = g_invQuantScales[ qpRem ] * TU::getBlockSizeTrafoScaleForDeQuant( area );
    if( shift < 0 )
    {
      invQScale <<= -shift;
      shift       = 0;
    }
    Intermediate_Int  add       = ( 1 << shift ) >> 1;

    //----- dequant coefficients -----
    for( int state = 0, scanIdx = lastScanIdx; scanIdx >= 0; scanIdx-- )
    {
      const unsigned  rasterPos = scan  [ scanIdx   ];
      const TCoeff&   level     = qCoeff[ rasterPos ];
      if( level )
      {
        Intermediate_Int  qIdx      = ( level << 1 ) + ( level > 0 ? -tab_nextQSet[ state ] : tab_nextQSet[ state ] );
        Intermediate_Int  nomTCoeff = ( qIdx * invQScale + add ) >> shift;
        tCoeff[ rasterPos ]         = (TCoeff)Clip3<Intermediate_Int>( minTCoeff, maxTCoeff, nomTCoeff );
      }
      state = tab_nextState[ state ][ level & 1 ];
    }
  }

  void Quantizer::xDequantStd( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff  )  const
  {
    CHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );

    //----- set basic parameters -----
    const CompArea&     area      = tu.blocks[ compID ];
    const int           numCoeff  = area.area();
    const TCoeff*       qCoeff    = tu.getCoeffs( compID ).buf;
          TCoeff*       tCoeff    = recCoeff.buf;

    //----- reset coefficients -----
    ::memset( tCoeff, 0, numCoeff * sizeof(TCoeff) );

    //----- set dequant parameters -----
    const int         qpPer                 = cQP.per;
    const int         qpRem                 = cQP.rem;
    const SPS&        sps                   = *tu.cs->sps;
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const TCoeff      minTCoeff             = -( 1 << maxLog2TrDynamicRange );
    const TCoeff      maxTCoeff             =  ( 1 << maxLog2TrDynamicRange ) - 1;
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );
    Intermediate_Int  shift     = IQUANT_SHIFT - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    Intermediate_Int  invQScale = g_invQuantScales[ qpRem ] * TU::getBlockSizeTrafoScaleForDeQuant( area );
    if( shift < 0 )
    {
      invQScale <<= -shift;
      shift       = 0;
    }
    Intermediate_Int  add       = ( 1 << shift ) >> 1;

    //----- dequant coefficients -----
    for( int k = 0; k < numCoeff; k++ )
    {
      const TCoeff& level = qCoeff[ k ];
      if( level )
      {
        Intermediate_Int  nomTCoeff = ( level * invQScale + add ) >> shift;
        tCoeff[ k ]                 = (TCoeff)Clip3<Intermediate_Int>( minTCoeff, maxTCoeff, nomTCoeff );
      }
    }
  }

  __inline void Quantizer::preQuantCoeff( const TCoeff absCoeff, PQData* pqData ) const
  {
    int64_t scaledOrg = int64_t( absCoeff ) * m_QScale;
    TCoeff  qIdx      = std::max<TCoeff>( 1, std::min<TCoeff>( m_maxQIdx, TCoeff( ( scaledOrg + m_QAdd ) >> m_QShift ) ) );
    int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;
    PQData& pq_a      = pqData[ qIdx & 3 ];
    pq_a.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_a.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_b      = pqData[ qIdx & 3 ];
    pq_b.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_b.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_c      = pqData[ qIdx & 3 ];
    pq_c.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_c.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_d      = pqData[ qIdx & 3 ];
    pq_d.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_d.absLevel     = ( ++qIdx ) >> 1;
  }







  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class State;

  struct SbbCtx
  {
    uint8_t*  sbbFlags;
    uint8_t*  levels;
  };

  class CommonCtx
  {
  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    __inline void swap()
    {
      std::swap( m_currSbbCtx, m_prevSbbCtx );
    }

    __inline void reset( const RateEstimator& rateEst )
    {
      m_nbInfo = rateEst.nbInfoOut();
      ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2*sizeof(BinFracBits) );
      const int numSbb    = rateEst.numSbb();
      const int chunkSize = numSbb + rateEst.numCoeff();
      uint8_t*  nextMem   = m_memory;
      for( int k = 0; k < 8; k++, nextMem += chunkSize )
      {
        m_allSbbCtx[k].sbbFlags = nextMem;
        m_allSbbCtx[k].levels   = nextMem + numSbb;
      }
    }

    __inline void update( const ScanInfo& scanInfo, const State* prevState, State& currState );

  private:
    const NbInfoOut*            m_nbInfo;
    BinFracBits                 m_sbbFlagBits[2];
    SbbCtx                      m_allSbbCtx  [8];
    SbbCtx*                     m_currSbbCtx;
    SbbCtx*                     m_prevSbbCtx;
    uint8_t                     m_memory[ 8 * ( MAX_TU_SIZE * MAX_TU_SIZE + MLS_GRP_NUM ) ];
  };


  class State
  {
    friend class CommonCtx;
  public:
    State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId );

    template<uint8_t numIPos>
    __inline void  updateState   ( const ScanInfo& scanInfo, const State* prevStates,                          const Decision& decision );
    __inline void  updateStateEOS( const ScanInfo& scanInfo, const State* prevStates, const State* skipStates, const Decision& decision );

    __inline void  init()
    {
      m_rdCost        = std::numeric_limits<int64_t>::max()>>1;
      m_numSigSbb     = 0;
      m_refSbbCtxId   = -1;
      m_sigFracBits   = m_sigFracBitsArray[ 0 ];
      m_coeffFracBits = m_gtxFracBitsArray[ 0 ];
      m_goRicePar     = 0;
    }

    template <ScanPosType spt>
    __inline void checkRdCostZero( Decision& decision ) const
    {
      int64_t rdCost = m_rdCost;
      if( spt == SCAN_ISCSBB )
      {
        rdCost += m_sigFracBits.intBits[0];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCost += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
      }
      else if( m_numSigSbb )
      {
        rdCost += m_sigFracBits.intBits[0];
      }
      else
      {
        return;
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = 0;
        decision.prevId   = m_stateId;
      }
    }

    __inline int32_t getLevelBits( const unsigned level ) const
    {
      if( level < 5 )
      {
        return m_coeffFracBits.bits[level];
      }
      int32_t   bits   = m_coeffFracBits.bits[5];
      unsigned  value   = level - 5;
      unsigned  thres   = g_auiGoRiceRange[ m_goRicePar ] << m_goRicePar;
      if( value < thres )
      {
        return bits + ( ( ( value >> m_goRicePar ) + 1 + m_goRicePar ) << SCALE_BITS );
      }
      unsigned  length  = m_goRicePar;
      unsigned  delta   = 1 << length;
      unsigned  valLeft = value - thres;
      while( valLeft >= delta )
      {
        valLeft -= delta;
        delta    = 1 << (++length);
      }
      return bits + ( ( g_auiGoRiceRange[ m_goRicePar ] + 1 + ( length << 1 ) - m_goRicePar ) << SCALE_BITS );
    }

    template <ScanPosType spt>
    __inline void checkRdCostNonZero( const PQData& pqData, Decision& decision )  const
    {
      int64_t rdCost = m_rdCost + pqData.deltaDist + getLevelBits( pqData.absLevel );
      if( spt == SCAN_ISCSBB )
      {
        rdCost += m_sigFracBits.intBits[1];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCost += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
      }
      else if( m_numSigSbb )
      {
        rdCost += m_sigFracBits.intBits[1];
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId   = m_stateId;
      }
    }

    __inline void checkRdCostStart( int32_t lastOffset, const PQData& pqData, Decision& decision ) const
    {
      int64_t rdCost = pqData.deltaDist + lastOffset + getLevelBits( pqData.absLevel );
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId   = -1;
      }
    }

    __inline void checkRdCostSkipSbb( Decision& decision )  const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = 0;
        decision.prevId   = 4+m_stateId;
      }
    }

  private:
    int64_t                   m_rdCost;
    uint8_t                   m_absLevelsAndCtxInit[32];  // 16 for abs levels + 16 for ctx init id
    int32_t                   m_numSigSbb;
    int32_t                   m_refSbbCtxId;
    BinFracBits               m_sbbFracBits;
    BinFracBits               m_sigFracBits;
    CoeffFracBits             m_coeffFracBits;
    int                       m_goRicePar;
    const int                 m_stateId;
    const BinFracBits*const   m_sigFracBitsArray;
    const CoeffFracBits*const m_gtxFracBitsArray;
    CommonCtx&                m_commonCtx;
  };


  State::State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId )
    : m_sbbFracBits     { { 0, 0 } }
    , m_stateId         ( stateId )
    , m_sigFracBitsArray( rateEst.sigFlagBits(stateId) )
    , m_gtxFracBitsArray( rateEst.gtxFracBits(stateId) )
    , m_commonCtx       ( commonCtx )
  {
  }


  template<uint8_t numIPos>
  __inline void State::updateState( const ScanInfo& scanInfo, const State* prevStates, const Decision& decision )
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      if( decision.prevId >= 0 )
      {
        const State*  prvState  = prevStates            +   decision.prevId;
        m_numSigSbb             = prvState->m_numSigSbb + !!decision.absLevel;
        m_refSbbCtxId           = prvState->m_refSbbCtxId;
        m_sbbFracBits           = prvState->m_sbbFracBits;
        ::memcpy( m_absLevelsAndCtxInit, prvState->m_absLevelsAndCtxInit, 32*sizeof(uint8_t) );
      }
      else
      {
        m_numSigSbb     =  1;
        m_refSbbCtxId   = -1;
        ::memset( m_absLevelsAndCtxInit, 0, 32*sizeof(uint8_t) );
      }
      m_absLevelsAndCtxInit[ scanInfo.insidePos ] = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

      TCoeff tinit  = m_absLevelsAndCtxInit[ 16 + scanInfo.nextInsidePos ];
      TCoeff sumAbs =   tinit >> 3;
      TCoeff sumGt1 = -(tinit  & 7);
      if( numIPos == 1 )
      {
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ] != 0;
      }
      else if( numIPos == 2 )
      {
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ] != 0;
      }
      else if( numIPos == 3 )
      {
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[2] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[2] ] != 0;
      }
      else if( numIPos == 4 )
      {
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[2] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[2] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[3] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[3] ] != 0;
      }
      else if( numIPos == 5 )
      {
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[0] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[1] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[2] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[2] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[3] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[3] ] != 0;
        sumAbs += m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[4] ];
        sumGt1 -= m_absLevelsAndCtxInit[ scanInfo.nextNbInfoSbb.inPos[4] ] != 0;
      }
      sumGt1         += sumAbs;
      m_sigFracBits   = m_sigFracBitsArray[ scanInfo.sigCtxOffsetNext + ( sumAbs < 5 ? sumAbs : 5 ) ];
      m_coeffFracBits = m_gtxFracBitsArray[ scanInfo.gtxCtxOffsetNext + ( sumGt1 < 4 ? sumGt1 : 4 ) ];
      m_goRicePar     = g_auiGoRiceTable  [ sumGt1 < 31 ? sumGt1 : 31 ];
    }
  }

  __inline void State::updateStateEOS( const ScanInfo& scanInfo, const State* prevStates, const State* skipStates, const Decision& decision )
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      const State* prvState = 0;
      if( decision.prevId  >= 0 )
      {
        prvState    = ( decision.prevId < 4 ? prevStates : skipStates - 4 ) +   decision.prevId;
        m_numSigSbb = prvState->m_numSigSbb                                 + !!decision.absLevel;
        ::memcpy( m_absLevelsAndCtxInit, prvState->m_absLevelsAndCtxInit, 16*sizeof(uint8_t) );
      }
      else
      {
        m_numSigSbb = 1;
        ::memset( m_absLevelsAndCtxInit, 0, 16*sizeof(uint8_t) );
      }
      m_absLevelsAndCtxInit[ scanInfo.insidePos ] = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

      m_commonCtx.update( scanInfo, prvState, *this );

      TCoeff  tinit   = m_absLevelsAndCtxInit[ 16 + scanInfo.nextInsidePos ];
      TCoeff  sumAbs  = tinit >> 3;
      TCoeff  sumGt1  = sumAbs - (tinit & 7);
      m_sigFracBits   = m_sigFracBitsArray[ scanInfo.sigCtxOffsetNext + ( sumAbs < 5 ? sumAbs : 5 ) ];
      m_coeffFracBits = m_gtxFracBitsArray[ scanInfo.gtxCtxOffsetNext + ( sumGt1 < 4 ? sumGt1 : 4 ) ];
      m_goRicePar     = g_auiGoRiceTable  [ sumGt1 < 31 ? sumGt1 : 31 ];
    }
  }



  __inline void CommonCtx::update( const ScanInfo& scanInfo, const State* prevState, State& currState )
  {
    uint8_t*    sbbFlags  = m_currSbbCtx[ currState.m_stateId ].sbbFlags;
    uint8_t*    levels    = m_currSbbCtx[ currState.m_stateId ].levels;
    std::size_t setCpSize = m_nbInfo[ scanInfo.scanIdx - 1 ].maxDist * sizeof(uint8_t);
    if( prevState && prevState->m_refSbbCtxId >= 0 )
    {
      ::memcpy( sbbFlags,                  m_prevSbbCtx[prevState->m_refSbbCtxId].sbbFlags,                  scanInfo.numSbb*sizeof(uint8_t) );
      ::memcpy( levels + scanInfo.scanIdx, m_prevSbbCtx[prevState->m_refSbbCtxId].levels + scanInfo.scanIdx, setCpSize );
    }
    else
    {
      ::memset( sbbFlags,                  0, scanInfo.numSbb*sizeof(uint8_t) );
      ::memset( levels + scanInfo.scanIdx, 0, setCpSize );
    }
    sbbFlags[ scanInfo.sbbPos ] = !!currState.m_numSigSbb;
    ::memcpy( levels + scanInfo.scanIdx, currState.m_absLevelsAndCtxInit, scanInfo.sbbSize*sizeof(uint8_t) );

    const int       sigNSbb   = ( ( scanInfo.nextSbbRight ? sbbFlags[ scanInfo.nextSbbRight ] : false ) || ( scanInfo.nextSbbBelow ? sbbFlags[ scanInfo.nextSbbBelow ] : false ) ? 1 : 0 );
    currState.m_numSigSbb     = 0;
    currState.m_refSbbCtxId   = currState.m_stateId;
    currState.m_sbbFracBits   = m_sbbFlagBits[ sigNSbb ];

    uint8_t           templateCtxInit[16];
    const int         scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
    const NbInfoOut*  nbOut     = m_nbInfo + scanBeg;
    const uint8_t*    absLevels = levels   + scanBeg;
    for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
    {
      if( nbOut->num )
      {
        TCoeff sumAbs = absLevels[ nbOut->outPos[0] ];
        TCoeff sumNum = absLevels[ nbOut->outPos[0] ] != 0;
        if( nbOut->num > 1 )
        {
          sumAbs += absLevels[ nbOut->outPos[1] ];
          sumNum += absLevels[ nbOut->outPos[1] ] != 0;
          if( nbOut->num > 2 )
          {
            sumAbs += absLevels[ nbOut->outPos[2] ];
            sumNum += absLevels[ nbOut->outPos[2] ] != 0;
            if( nbOut->num > 3 )
            {
              sumAbs += absLevels[ nbOut->outPos[3] ];
              sumNum += absLevels[ nbOut->outPos[3] ] != 0;
              if( nbOut->num > 4 )
              {
                sumAbs += absLevels[ nbOut->outPos[4] ];
                sumNum += absLevels[ nbOut->outPos[4] ] != 0;
              }
            }
          }
        }
        templateCtxInit[id] = uint8_t(sumNum) + ( (uint8_t)std::min<TCoeff>( 31, sumAbs ) << 3 );
      }
      else
      {
        templateCtxInit[id] = 0;
      }
    }
    ::memset( currState.m_absLevelsAndCtxInit,      0,               16*sizeof(uint8_t) );
    ::memcpy( currState.m_absLevelsAndCtxInit + 16, templateCtxInit, 16*sizeof(uint8_t) );
  }







  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q                                                              =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/
  class TCQ : private RateEstimator
  {
  public:
    TCQ();

    void    quant   ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum );
    void    dequant ( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP )  const;

  private:
    void    xDecideAndUpdate  ( const TCoeff absCoeff, const ScanInfo& scanInfo );
    template<ScanPosType spt>
    void    xDecide           ( const TCoeff absCoeff, int32_t lastOffset, Decision* decisions );

  private:
    CommonCtx   m_commonCtx;
    State       m_allStates[ 12 ];
    State*      m_currStates;
    State*      m_prevStates;
    State*      m_skipStates;
    State       m_startState;
    Quantizer   m_quant;
    Decision    m_trellis[ MAX_TU_SIZE * MAX_TU_SIZE ][ 8 ];
  };


#define TINIT(x) {*this,m_commonCtx,x}
  TCQ::TCQ()
    : RateEstimator ()
    , m_commonCtx   ()
    , m_allStates   {TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3)}
    , m_currStates  (  m_allStates      )
    , m_prevStates  (  m_currStates + 4 )
    , m_skipStates  (  m_prevStates + 4 )
    , m_startState  TINIT(0)
  {}
#undef TINIT


  void TCQ::dequant( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP ) const
  {
    m_quant.dequantBlock( tu, compID, cQP, recCoeff, tu.tcq[compID] );
  }


#define DINIT(l,p) {std::numeric_limits<int64_t>::max()>>2,l,p}
  static const Decision startDec[8] = {DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(0,4),DINIT(0,5),DINIT(0,6),DINIT(0,7)};
#undef  DINIT


  template<ScanPosType spt>
  void TCQ::xDecide( const TCoeff absCoeff, int32_t lastOffset, Decision* decisions )
  {
    ::memcpy( decisions, startDec, 8*sizeof(Decision) );

    PQData  pqData[4];
    m_quant.preQuantCoeff( absCoeff, pqData );
    m_prevStates[0].checkRdCostNonZero<spt> ( pqData[0],  decisions[0] );
    m_prevStates[0].checkRdCostNonZero<spt> ( pqData[2],  decisions[2] );
    m_prevStates[0].checkRdCostZero<spt>                ( decisions[0] );
    m_prevStates[1].checkRdCostNonZero<spt> ( pqData[2],  decisions[0] );
    m_prevStates[1].checkRdCostNonZero<spt> ( pqData[0],  decisions[2] );
    m_prevStates[1].checkRdCostZero<spt>                ( decisions[2] );
    m_prevStates[2].checkRdCostNonZero<spt> ( pqData[3],  decisions[1] );
    m_prevStates[2].checkRdCostNonZero<spt> ( pqData[1],  decisions[3] );
    m_prevStates[2].checkRdCostZero<spt>                ( decisions[1] );
    m_prevStates[3].checkRdCostNonZero<spt> ( pqData[1],  decisions[1] );
    m_prevStates[3].checkRdCostNonZero<spt> ( pqData[3],  decisions[3] );
    m_prevStates[3].checkRdCostZero<spt>                ( decisions[3] );
    if( spt==SCAN_EOCSBB )
    {
      m_skipStates[0].checkRdCostSkipSbb( decisions[0] );
      m_skipStates[1].checkRdCostSkipSbb( decisions[1] );
      m_skipStates[2].checkRdCostSkipSbb( decisions[2] );
      m_skipStates[3].checkRdCostSkipSbb( decisions[3] );
    }
    m_startState.checkRdCostStart( lastOffset, pqData[0], decisions[0] );
    m_startState.checkRdCostStart( lastOffset, pqData[2], decisions[2] );
  }

  void TCQ::xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo )
  {
    Decision* decisions = m_trellis[ scanInfo.scanIdx ];

    std::swap( m_prevStates, m_currStates );

    if     ( scanInfo.socsbb )  { xDecide<SCAN_SOCSBB>( absCoeff, scanInfo.lastOffset, decisions ); }
    else if( scanInfo.eocsbb )  { xDecide<SCAN_EOCSBB>( absCoeff, scanInfo.lastOffset, decisions ); }
    else                        { xDecide<SCAN_ISCSBB>( absCoeff, scanInfo.lastOffset, decisions ); }

    if( scanInfo.scanIdx )
    {
      if( scanInfo.eosbb )
      {
        m_commonCtx.swap();
        m_currStates[0].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[0] );
        m_currStates[1].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[1] );
        m_currStates[2].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[2] );
        m_currStates[3].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[3] );
        ::memcpy( decisions+4, decisions, 4*sizeof(Decision) );
      }
      else
      {
        switch( scanInfo.nextNbInfoSbb.num )
        {
        case 0:
          m_currStates[0].updateState<0>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<0>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<0>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<0>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 1:
          m_currStates[0].updateState<1>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<1>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<1>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<1>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 2:
          m_currStates[0].updateState<2>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<2>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<2>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<2>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 3:
          m_currStates[0].updateState<3>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<3>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<3>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<3>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 4:
          m_currStates[0].updateState<4>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<4>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<4>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<4>( scanInfo, m_prevStates, decisions[3] );
          break;
        default:
          m_currStates[0].updateState<5>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<5>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<5>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<5>( scanInfo, m_prevStates, decisions[3] );
        }
      }

      if( scanInfo.socsbb )
      {
        std::swap( m_prevStates, m_skipStates );
      }
    }
  }


  void TCQ::quant( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum )
  {
    //===== reset / pre-init =====
    RateEstimator::initBlock  ( tu, compID );
    m_quant.initQuantBlock    ( tu, compID, cQP, lambda, true );
    TCoeff*       qCoeff      = tu.getCoeffs( compID ).buf;
    const TCoeff* tCoeff      = srcCoeff.buf;
    const int     numCoeff    = tu.blocks[compID].area();
    ::memset( tu.getCoeffs( compID ).buf, 0x00, numCoeff*sizeof(TCoeff) );
    absSum          = 0;
    tu.tcq[compID]  = false;

    //===== find first test position =====
    int   firstTestPos = numCoeff - 1;
    const TCoeff thres = m_quant.getLastThreshold();
    for( ; firstTestPos >= 0; firstTestPos-- )
    {
      if( abs( tCoeff[ rasterPos(firstTestPos) ] ) > thres )
      {
        break;
      }
    }
    if( firstTestPos < 0 )
    {
      return;
    }

    //===== real init =====
    RateEstimator::initCtx( tu, ctx.getFracBitsAcess() );
    m_commonCtx.reset( *this );
    for( int k = 0; k < 12; k++ )
    {
      m_allStates[k].init();
    }
    m_startState.init();


    //===== populate trellis =====
    for( ScanData scanData(*this,firstTestPos); scanData.valid(); scanData.next() )
    {
      xDecideAndUpdate( abs( tCoeff[ scanData.rasterPos ] ), scanData );
    }

    //===== find best path =====
    Decision  decision    = { std::numeric_limits<int64_t>::max(), -1, -2 };
    int64_t   minPathCost =  0;
    for( int8_t stateId = 0; stateId < 4; stateId++ )
    {
      int64_t pathCost = m_trellis[0][stateId].rdCost;
      if( pathCost < minPathCost )
      {
        decision.prevId = stateId;
        minPathCost     = pathCost;
      }
    }

    //===== backward scanning =====
    int scanIdx = 0;
    for( ; decision.prevId >= 0; scanIdx++ )
    {
      decision          = m_trellis[ scanIdx ][ decision.prevId ];
      int32_t blkpos    = rasterPos( scanIdx );
      qCoeff[ blkpos ]  = ( tCoeff[ blkpos ] < 0 ? -decision.absLevel : decision.absLevel );
      absSum           += decision.absLevel;
    }
    tu.tcq[compID] = ( absSum != 0 );
  }

}; // namespace TCQIntern




//===== interface class =====
TCQ::TCQ( const Quant* other ) : QuantRDOQ2( other )
{
  p = new TCQIntern::TCQ();
}

TCQ::~TCQ()
{
  delete static_cast<TCQIntern::TCQ*>(p);
}

void TCQ::init( UInt uiMaxTrSize, Bool useRDOQ, Bool useRDOQTS, UInt uiAltResiCompId,
#if T0196_SELECTIVE_RDOQ
                Bool useSelectiveRDOQ
#endif
                )
{
  QuantRDOQ2::init( uiMaxTrSize, useRDOQ, useRDOQTS, uiAltResiCompId,
#if T0196_SELECTIVE_RDOQ
                        useSelectiveRDOQ
#endif
  );
}

void TCQ::quant( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx )
{
  static_cast<TCQIntern::TCQ*>(p)->quant( tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, uiAbsSum );
}

void TCQ::dequant( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP )
{
  static_cast<TCQIntern::TCQ*>(p)->dequant( tu, dstCoeff, compID, cQP );
}




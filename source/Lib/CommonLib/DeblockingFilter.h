/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     DeblockingFilter.h
    \brief    deblocking filter (header)
*/

#ifndef __DEBLOCKINGFILTER__
#define __DEBLOCKINGFILTER__

#include "CommonDef.h"
#include "Unit.h"
#include "Picture.h"

//! \ingroup CommonLib
//! \{

#define DEBLOCK_SMALLEST_BLOCK  8

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// deblocking filter class
class DeblockingFilter
{
  static constexpr int LOG_GRID_SIZE = 2;
  static constexpr int GRID_SIZE     = 1 << LOG_GRID_SIZE;

private:
  static_vector<char, MAX_NUM_PARTS_IN_CTU> m_boundaryStrengths[NUM_EDGE_DIR];   // Bs for [Ver/Hor][Blk_Idx]
  static_vector<bool, MAX_NUM_PARTS_IN_CTU> m_edgeFilterFlags[NUM_EDGE_DIR];
  LFCUParam m_stLFCUParam;                   ///< status structure
  int     m_ctuXLumaSamples, m_ctuYLumaSamples;                            // location of left-edge and top-edge of CTU
  int     m_shiftHor, m_shiftVer;                                          // shift values to convert location from luma sample units to chroma sample units

  // maxFilterLengthP for [component][luma/chroma sample distance from left edge of CTU]
  // [luma/chroma sample distance from top edge of CTU]
  uint8_t m_maxFilterLengthP[MAX_NUM_COMPONENT][MAX_CU_SIZE / GRID_SIZE][MAX_CU_SIZE / GRID_SIZE];

  // maxFilterLengthQ for [component][luma/chroma sample distance from left edge of CTU]
  // [luma/chroma sample distance from top edge of CTU]
  uint8_t m_maxFilterLengthQ[MAX_NUM_COMPONENT][MAX_CU_SIZE / GRID_SIZE][MAX_CU_SIZE / GRID_SIZE];

  // transform edge flag for [component][luma/chroma sample distance from left edge of CTU]
  // [luma/chroma sample distance from top edge of CTU]
  bool m_transformEdge[MAX_NUM_COMPONENT][MAX_CU_SIZE / GRID_SIZE][MAX_CU_SIZE / GRID_SIZE];

  PelStorage                   m_encPicYuvBuffer;
  bool                         m_enc;
private:
  void clearFilterLengthAndTransformEdge();

  // set / get functions
  void xSetDeblockingFilterParam        ( const CodingUnit& cu );

  // filtering functions
  unsigned
  xGetBoundaryStrengthSingle      ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const Position& localPos, const ChannelType chType  ) const;

  void xSetEdgefilterMultiple(const CodingUnit &cu, const DeblockEdgeDir edgeDir, const Area &area, const bool value,
                              const bool edgeIdx = false);
  void xEdgeFilterLuma(const CodingUnit &cu, const DeblockEdgeDir edgeDir, const int edgeIdx);
  void xEdgeFilterChroma(const CodingUnit &cu, const DeblockEdgeDir edgeDir, const int edgeIdx);

  int deriveLADFShift(const Pel *src, const int stride, const DeblockEdgeDir edgeDir, const SPS *sps);
  void xSetMaxFilterLengthPQFromTransformSizes(const DeblockEdgeDir edgeDir, const CodingUnit &cu,
                                               const TransformUnit &currTU, const int firstComponent);
  void xSetMaxFilterLengthPQForCodingSubBlocks( const DeblockEdgeDir edgeDir, const CodingUnit& cu, const PredictionUnit& currPU, const bool& mvSubBlocks, const int& subBlockSize, const Area& areaPu );

  inline void xBilinearFilter     ( Pel* srcP, Pel* srcQ, int offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc ) const;
  inline void xFilteringPandQ     ( Pel* src, int offset, int numberPSide, int numberQSide, int tc ) const;
  inline void xPelFilterLuma(Pel *src, const int offset, const int tc, const bool sw, const bool partPNoFilter,
                             const bool partQNoFilter, const int thrCut, const bool bFilterSecondP,
                             const bool bFilterSecondQ, const ClpRng &clpRng, bool sidePisLarge = false,
                             bool sideQisLarge = false, int maxFilterLengthP = 7, int maxFilterLengthQ = 7) const;
  inline void xPelFilterChroma(Pel *src, const int offset, const int tc, const bool sw, const bool partPNoFilter,
                               const bool partQNoFilter, const ClpRng &clpRng, const bool largeBoundary,
                               const bool isChromaHorCTBBoundary) const;

  inline bool xUseStrongFiltering(Pel *src, const int offset, const int d, const int beta, const int tc,
                                  bool sidePisLarge = false, bool sideQisLarge = false, int maxFilterLengthP = 7,
                                  int maxFilterLengthQ = 7, bool isChromaHorCTBBoundary = false) const;

  inline unsigned BsSet(unsigned val, const ComponentID compIdx) const;
  inline unsigned BsGet(unsigned val, const ComponentID compIdx) const;

  inline bool isCrossedByVirtualBoundaries ( const int xPos, const int yPos, const int width, const int height, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], const PicHeader* picHeader );
  inline void xDeriveEdgefilterParam       ( const int xPos, const int yPos, const int numVerVirBndry, const int numHorVirBndry, const int verVirBndryPos[], const int horVirBndryPos[], bool &verEdgeFilter, bool &horEdgeFilter );

  inline int xCalcDP(Pel *src, const int offset, const bool isChromaHorCTBBoundary = false) const;
  inline int xCalcDQ(Pel *src, const int offset) const;

  static const uint16_t sm_tcTable[MAX_QP + 3];
  static const uint8_t sm_betaTable[MAX_QP + 1];

public:

  DeblockingFilter();
  ~DeblockingFilter();

  /// CU-level deblocking function
  void xDeblockCU(CodingUnit& cu, const DeblockEdgeDir edgeDir);
  void  initEncPicYuvBuffer(ChromaFormat chromaFormat, const Size &size, const unsigned maxCUSize);
  PelStorage& getDbEncPicYuvBuffer() { return m_encPicYuvBuffer; }
  void  setEnc(bool b) { m_enc = b; }

  void  create(const unsigned maxCUDepth);
  void  destroy                   ();

  /// picture-level deblocking filter
  void deblockingFilterPic        ( CodingStructure& cs );

  static int getBeta              ( const int qp )
  {
    const int indexB = Clip3( 0, MAX_QP, qp );
    return sm_betaTable[ indexB ];
  }

  void resetBsAndEdgeFilter(int edgeDir);
  void resetFilterLengths();
};

//! \}

#endif

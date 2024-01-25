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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__

// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"
// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class InterPrediction : public WeightPrediction
{
private:
  // Private member variables go here (not provided in the snippet).

protected:
  InterpolationFilter  m_if; // Instância de um filtro de interpolação.

  // Arrays para armazenar blocos preditos em YUV.
  Pel*                 m_acYuvPred[NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];

  // Arrays para armazenar blocos filtrados.
  Pel*                 m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL]
                                      [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL]
                                      [MAX_NUM_COMPONENT];
  
  Pel*                 m_filteredBlockTmp[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL]
                                         [MAX_NUM_COMPONENT];

  static constexpr int TMP_RPR_WIDTH  = MAX_CU_SIZE + 16;
  static constexpr int TMP_RPR_HEIGHT = MAX_CU_SIZE * MAX_SCALING_RATIO + 16;

  Pel *m_filteredBlockTmpRPR; // Buffer temporário para armazenar blocos filtrados.

  ChromaFormat         m_currChromaFormat; // Formato de croma atual.

  ComponentID          m_maxCompIDToPred; // ID do componente máximo a ser predito.

  RdCost*              m_pcRdCost; // Custo RD para otimização.

  int                  m_iRefListIdx; // Índice da lista de referência.

  PelStorage           m_geoPartBuf[2]; // Buffer para armazenar partes geométricas.

  static constexpr int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;

  Mv*                  m_storedMv; // Vetor de movimento armazenado.

  // Buffers para dados do filtro bilinear para refinamento DMVR.
  Pel*                 m_cYuvPredTempDMVRL0;
  Pel*                 m_cYuvPredTempDMVRL1;
  int                  m_biLinearBufStride;

  // Buffers para dados acolchoados.
  PelUnitBuf           m_cYuvRefBuffDMVRL0;
  PelUnitBuf           m_cYuvRefBuffDMVRL1;
  Pel*                 m_cRefSamplesDMVRL0[MAX_NUM_COMPONENT];
  Pel*                 m_cRefSamplesDMVRL1[MAX_NUM_COMPONENT];

  // Vetor de compensação de busca para DMVR.
  Mv m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                             Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                             Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                             Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                             Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };

  // Array para armazenar SADs para iterações DMVR.
  uint64_t m_SADsArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];

  static constexpr int AFFINE_SUBBLOCK_WIDTH_EXT  = AFFINE_SUBBLOCK_SIZE + 2 * PROF_BORDER_EXT_W;
  static constexpr int AFFINE_SUBBLOCK_HEIGHT_EXT = AFFINE_SUBBLOCK_SIZE + 2 * PROF_BORDER_EXT_H;

  // Buffer para armazenar gradientes.
  Pel m_gradBuf[2][AFFINE_SUBBLOCK_WIDTH_EXT * AFFINE_SUBBLOCK_HEIGHT_EXT];

  // PROF skip flags for encoder speedup
  bool m_skipProf{ false }; // Sinalizador de salto para otimização de velocidade do codificador.
  bool m_skipProfCond{ false }; // Sinalizador de condição de salto para otimização de velocidade do codificador.
  bool m_biPredSearchAffine{ false }; // Sinalizador para pesquisa de previsão bilateral para afinamento.

  Pel* m_gradX0; // Buffer para gradientes na direção X (para bio).
  Pel* m_gradY0; // Buffer para gradientes na direção Y (para bio).
  Pel* m_gradX1; // Buffer para gradientes na direção X (para bio).
  Pel* m_gradY1; // Buffer para gradientes na direção Y (para bio).
  bool m_subPuMC; // Sinalizador para Sub PU MC (Motion Compensation).

  int m_IBCBufferWidth; // Largura do buffer IBC (Intra Block Copy).
  PelStorage m_IBCBuffer; // Buffer IBC.

  // Função para cópia intra bloco.
  void xIntraBlockCopy(PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID);

  // Função para deslocamento à direita do bit mais significativo.
  int rightShiftMSB(int numer, int denom);

  // Função para aplicar o fluxo bidirecional otimizado.
  void applyBiOptFlow(const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1,
                      const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths);

  // Função para realizar a previsão unidirecional.
  void xPredInterUni(const PredictionUnit &pu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred, const bool bi,
                     const bool bioApplied, const bool luma, const bool chroma);

  // Função para realizar a previsão bidirecional.
  void xPredInterBi(PredictionUnit &pu, PelUnitBuf &pcYuvPred, bool luma, bool chroma, PelUnitBuf *yuvPredTmp);

  // Função para realizar a previsão de um bloco intermediário.
  void xPredInterBlk(const ComponentID compID, const PredictionUnit &pu, const Picture *refPic, const Mv &_mv,
                     PelUnitBuf &dstPic, bool bi, const ClpRng &clpRng, bool bioApplied, bool isIBC,
                     const std::pair<int, int> scalingRatio, SizeType dmvrWidth, SizeType dmvrHeight, bool bilinearMC,
                     Pel *srcPadBuf, int32_t srcPadStride);

  // Sobrecarga da função para realizar a previsão de um bloco intermediário.
  void xPredInterBlk(const ComponentID compID, const PredictionUnit &pu, const Picture *refPic, const Mv &_mv,
                     PelUnitBuf &dstPic, bool bi, const ClpRng &clpRng, bool bioApplied, bool isIBC)
  {
    xPredInterBlk(compID, pu, refPic, _mv, dstPic, bi, clpRng, bioApplied, isIBC, SCALE_1X, 0, 0, false, nullptr, 0);
  }

  // Função para adicionar a média ponderada usando BIO para 4 pixels.
  void xAddBIOAvg4(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride,
                   const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride,
                   int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng);

  // Função para realizar o filtro de gradiente BIO.
  void xBioGradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, int bitDepth);

  // Função para calcular os parâmetros BIO.
  void xCalcBIOPar(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1,
                   const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2,
                   int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride,
                   const int src1Stride, const int gradStride, const int widthG, const int heightG, int bitDepth);

  // Função para calcular o gradiente de um bloco.
  void xCalcBlkGradient(int sx, int sy, int *arraysGx2, int *arraysGxGy, int *arraysGxdI, int *arraysGy2,
                        int *arraysGydI, int &sGx2, int &sGy2, int &sGxGy, int &sGxdI, int &sGydI, int width,
                        int height, int unitSize);

  // Função para realizar a média ponderada.
  void xWeightedAverage(const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1,
                        PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths, const ClpRngs &clpRngs,
                        bool bioApplied, bool lumaOnly, bool chromaOnly, PelUnitBuf *yuvDstTmp);

#if GDR_ENABLED
  // Função para realizar a previsão de bloco affine.
  bool xPredAffineBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic, const Mv *_mv,
                      PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng, const bool genChromaMv = false,
                      const std::pair<int, int> scalingRatio = SCALE_1X);
#else
  // Sobrecarga da função para realizar a previsão de bloco affine.
  void xPredAffineBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic, const Mv *_mv,
                      PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng, const bool genChromaMv = false,
                      const std::pair<int, int> scalingRatio = SCALE_1X);
#endif

  // Função estática para verificar se o movimento é idêntico.
  static bool xCheckIdenticalMotion(const PredictionUnit& pu);

  // Função para realizar a sub-PU MC (Motion Compensation).
  void xSubPuMC(PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList, bool luma, bool chroma);

  // Função para realizar a sub-PU BIO.
  void xSubPuBio(PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList, PelUnitBuf *yuvDstTmp);

  // Função para destruir objetos.
  void destroy();

  MotionInfo m_SubPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)]; // Buffer para informações de movimento.

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel *m_cacheModel; // Modelo de cache para medição de largura de banda de memória.
#endif
  PelStorage m_colorTransResiBuf[3]; // Buffers para resíduos de transformação de cor (0-org; 1-act; 2-tmp).

public:
  InterPrediction(); // Construtor da classe.
  virtual ~InterPrediction(); // Destrutor da classe.

  // Função de inicialização.
  void init(RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize);

  // Função de compensação de movimento.
  void motionCompensation(PredictionUnit &pu, PelUnitBuf &predBuf, RefPicList eRefPicList, bool luma, bool chroma,
                          PelUnitBuf *predBufWOBIO);

  // Sobrecarga da função de compensação de movimento para luma e croma.
  void motionCompensation(PredictionUnit &pu, PelUnitBuf &predBuf, RefPicList eRefPicList)
  {
    motionCompensation(pu, predBuf, eRefPicList, true, true, nullptr);
  }

  // Função para realizar a compensação de movimento em uma PU.
  void motionCompensatePu(PredictionUnit &pu, RefPicList eRefPicList, bool luma, bool chroma);

  // Função para realizar a compensação de movimento em um CU.
  void motionCompensateCu(CodingUnit &cu, RefPicList eRefPicList, bool luma, bool chroma);

  // Função para realizar a compensação de movimento geométrica.
  void motionCompensationGeo(CodingUnit &cu, MergeCtx &GeoMrgCtx);

  // Função para realizar a média ponderada geométrica em um bloco.
  void weightedGeoBlk(PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst,
                      PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);

  // Função para pré-carregar dados em uma PU para previsão.
  void xPrefetch(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma);

  // Função para realizar o acolchoamento em uma PU.
  void xPad(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId);

  // Função para realizar a compensação de movimento final para DMVR (DMMVR).
  void xFinalPaddedMCForDMVR(PredictionUnit &pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0,
                                PelUnitBuf &pcPad1, const bool bioApplied, const Mv startMV[NUM_REF_PIC_LIST_01],
                                bool blockMoved);

  // Função para refinar o vetor de movimento usando DMVR.
  void xBIPMVRefine(int bd, Pel *pRefL0, Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray,
                    int width, int height);

  // Função para calcular o custo DMVR.
  uint64_t xDMVRCost(int bitDepth, Pel* pRef, uint32_t refStride, const Pel* pOrg, uint32_t orgStride, int width, int height);

  // Função para inicializar a compensação de movimento.
  void xinitMC(PredictionUnit& pu, const ClpRngs &clpRngs);

  // Função para processar DMVR.
  void xProcessDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  // Função para atribuir um modelo de cache para medição de largura de banda de memória.
  void cacheAssign( CacheModel *cache );
#endif

  // Função estática para verificar se o vetor de subbloco está espalhado além do limite.
  static bool isSubblockVectorSpreadOverLimit(int a, int b, int c, int d, int predType);

  // Função para preencher o buffer IBC.
  void xFillIBCBuffer(CodingUnit &cu);

  // Função para redefinir o buffer IBC.
  void resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize);

  // Função para redefinir VPDU para IBC.
  void resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos,
                       const int yPos);

  // Função para verificar se um vetor de movimento luma é válido.
  bool isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const

  // Função para realizar a previsão de bloco RPR (Region Partitioning and Refinement).
  bool xPredInterBlkRPR(const std::pair<int, int> &scalingRatio, const PPS &pps, const CompArea &blk,
                        const Picture *refPic, const Mv &mv, Pel *dst, const int dstStride, const bool bi,
                        const bool wrapRef, const ClpRng &clpRng, const InterpolationFilter::Filter filterIndex,
                        const bool useAltHpelIf = false);
};

//! \}

#endif // __INTERPREDICTION__

/*________________________________________________________________________________________________________________________________
Explicação:
m_skipProf, m_skipProfCond, e m_biPredSearchAffine são sinalizadores que indicam otimizações para a velocidade do codificador.
m_gradX0, m_gradY0, m_gradX1, e m_gradY1 são buffers para gradientes usados na aplicação do fluxo bidirecional otimizado (BIO).
m_subPuMC é um sinalizador para indicar se a sub-PU MC (Motion Compensation) deve ser realizada.
m_IBCBufferWidth é a largura do buffer IBC (Intra Block Copy).
m_IBCBuffer é o buffer IBC usado para Intra Block Copy.
xIntraBlockCopy é uma função para realizar a cópia intra bloco.
rightShiftMSB é uma função para realizar o deslocamento à direita do bit mais significativo.
applyBiOptFlow é uma função para aplicar o fluxo bidirecional otimizado.
xPredInterUni é uma função para realizar a previsão unidirecional.
xPredInterBi é uma função para realizar a previsão bidirecional.
xPredInterBlk é uma função para realizar a previsão de um bloco intermediário.
xAddBIOAvg4 é uma função para adicionar a média ponderada usando BIO para 4 pixels.
xBioGradFilter é uma função para realizar o filtro de gradiente BIO.
xCalcBIOPar é uma função para calcular os parâmetros BIO.
xCalcBlkGradient é uma função para calcular o gradiente de um bloco.
xWeightedAverage é uma função para realizar a média ponderada.
xCheckIdenticalMotion é uma função estática para verificar se o movimento é idêntico.
xSubPuMC é uma função para realizar a sub-PU MC.
xSubPuBio é uma função para realizar a sub-PU BIO.
destroy é uma função para destruir objetos.
m_SubPuMiBuf é um buffer para informações de movimento.
m_cacheModel é um modelo de cache para medição de largura de banda de memória.
m_colorTransResiBuf são buffers para resíduos de transformação de cor.
InterPrediction é o construtor da classe.
~InterPrediction é o destrutor da classe.
init é uma função de inicialização.
motionCompensation é uma função para realizar a compensação de movimento.
motionCompensatePu é uma função para realizar a compensação de movimento em uma PU.
motionCompensateCu é uma função para realizar a compensação de movimento em um CU.
motionCompensationGeo é uma função para realizar a compensação de movimento geométrica.
weightedGeoBlk é uma função para realizar a média ponderada geométrica em um bloco.
xPrefetch é uma função para pré-carregar dados em uma PU para previsão.
xPad é uma função para realizar o acolchoamento em uma PU.
xFinalPaddedMCForDMVR é uma função para realizar a compensação de movimento final para DMVR.
xBIPMVRefine é uma função para refinar o vetor de movimento usando DMVR.
xDMVRCost é uma função para calcular o custo DMVR.
xinitMC é uma função para inicializar a compensação de movimento.
xProcessDMVR é uma função para processar DMVR.
isSubblockVectorSpreadOverLimit é uma função estática para verificar se o vetor de subbloco está espalhado além do limite.
xFillIBCBuffer é uma função para preencher o buffer IBC.
resetIBCBuffer é uma função para redefinir o buffer IBC.
resetVPDUforIBC é uma função para redefinir VPDU para IBC.
isLumaBvValid é uma função para verificar se um vetor de movimento luma é válido.
xPredInterBlkRPR é uma função para realizar a previsão de bloco RPR.
________________________________________________________________________________________________________________________________*/
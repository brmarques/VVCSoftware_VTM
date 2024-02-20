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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"
#include "MCTS.h"

#include <memory.h>
#include <algorithm>

#include <iostream>
#include <chrono>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
  : m_currChromaFormat(NUM_CHROMA_FORMAT)
  , m_maxCompIDToPred(MAX_NUM_COMPONENT)
  , m_pcRdCost(nullptr)
  , m_storedMv(nullptr)
  , m_gradX0(nullptr)
  , m_gradY0(nullptr)
  , m_gradX1(nullptr)
  , m_gradY1(nullptr)
  , m_subPuMC(false)
  , m_IBCBufferWidth(0)
{
  // Inicializa membros da classe com valores padrão ou nulos
  // Inicialização dos arrays de ponteiros para armazenar previsões de pixel de referência
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  // Inicialização dos arrays de ponteiros para armazenar blocos filtrados
  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

  // Inicialização de outros membros da classe
  m_cYuvPredTempDMVRL1 = nullptr;
  m_cYuvPredTempDMVRL0 = nullptr;

  // Inicialização dos arrays de ponteiros para amostras de referência utilizadas no processo de fusão de movimento
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    m_cRefSamplesDMVRL0[ch] = nullptr;
    m_cRefSamplesDMVRL1[ch] = nullptr;
  }
}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  // Libera recursos alocados para previsões de pixel de referência
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  // Libera recursos alocados para blocos filtrados
  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

  // Libera recursos de buffers específicos
  m_geoPartBuf[0].destroy();
  m_geoPartBuf[1].destroy();
  m_colorTransResiBuf[0].destroy();
  m_colorTransResiBuf[1].destroy();
  m_colorTransResiBuf[2].destroy();

  // Libera recursos alocados para outros membros da classe
  if (m_storedMv != nullptr)
  {
    delete[] m_storedMv;
    m_storedMv = nullptr;
  }

  xFree(m_gradX0);   m_gradX0 = nullptr;
  xFree(m_gradY0);   m_gradY0 = nullptr;
  xFree(m_gradX1);   m_gradX1 = nullptr;
  xFree(m_gradY1);   m_gradY1 = nullptr;

  xFree(m_filteredBlockTmpRPR);
  m_filteredBlockTmpRPR = nullptr;

  xFree(m_cYuvPredTempDMVRL0);
  m_cYuvPredTempDMVRL0 = nullptr;
  xFree(m_cYuvPredTempDMVRL1);
  m_cYuvPredTempDMVRL1 = nullptr;

  // Libera recursos alocados para amostras de referência usadas no processo de fusão de movimento
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    xFree(m_cRefSamplesDMVRL0[ch]);
    m_cRefSamplesDMVRL0[ch] = nullptr;
    xFree(m_cRefSamplesDMVRL1[ch]);
    m_cRefSamplesDMVRL1[ch] = nullptr;
  }

  // Libera recursos alocados para um buffer específico
  m_IBCBuffer.destroy();
}

void InterPrediction::init(RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize)
{
  // Armazena o ponteiro para o objeto RdCost, que é usado para cálculos de custo durante a predição.
  m_pcRdCost = pcRdCost;

  // Se a classe já foi inicializada antes e o formato de croma mudou, libera a memória e reinicia.
  if (m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();  // Libera recursos se necessário.
  }

  // Atualiza o formato de croma atual.
  m_currChromaFormat = chromaFormatIDC;

  // Verifica se a classe ainda não foi inicializada (pela primeira vez).
  if (m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr)
  {
    // Inicializa as estruturas de dados e aloca memória para predição e filtragem.

    // Aloca memória para os blocos filtrados.
    for (uint32_t c = 0; c < MAX_NUM_COMPONENT; c++)
    {
      int extWidth = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 16;
      int extHeight = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 1;
      extWidth = extWidth > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16) ? extWidth : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16;
      extHeight = extHeight > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1) ? extHeight : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1;

      for (uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++)
      {
        m_filteredBlockTmp[i][c] = (Pel*)xMalloc(Pel, (extWidth + 4) * (extHeight + 7 + 4));

        for (uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++)
        {
          m_filteredBlock[i][j][c] = (Pel*)xMalloc(Pel, extWidth * extHeight);
        }
      }

      // Aloca memória para a predição.
      for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
      {
        m_acYuvPred[i][c] = (Pel*)xMalloc(Pel, MAX_CU_SIZE * MAX_CU_SIZE);
      }
    }

    // Cria buffers para diversas finalidades.
    m_geoPartBuf[0].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_geoPartBuf[1].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_colorTransResiBuf[0].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_colorTransResiBuf[1].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_colorTransResiBuf[2].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

    // Inicializa ponteiros para gradientes e outros buffers.
    m_gradX0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradX1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);

    // Aloca memória para o bloco filtrado temporário para a técnica RPR (RefinePelResampling).
    m_filteredBlockTmpRPR = (Pel*)xMalloc(Pel, TMP_RPR_WIDTH * TMP_RPR_HEIGHT);
  }

  // Se os buffers para a técnica DMVR ainda não foram alocados, aloca-os.
  if (m_cYuvPredTempDMVRL0 == nullptr && m_cYuvPredTempDMVRL1 == nullptr)
  {
    m_cYuvPredTempDMVRL0 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)));
    m_cYuvPredTempDMVRL1 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)));

    // Aloca memória para amostras de referência usadas na técnica DMVR.
    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      m_cRefSamplesDMVRL0[ch] = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA));
      m_cRefSamplesDMVRL1[ch] = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA));
    }
  }

  // Inicializa o filtro de interpolação se não estiver medindo o consumo de memória.
#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter(true);
#endif

  // Aloca memória para armazenar Mv (Motion Vectors) se ainda não foi alocada.
  if (m_storedMv == nullptr)
  {
    m_storedMv = new Mv[MVBUFFER_SIZE * MVBUFFER_SIZE];
  }

  // Se a largura do buffer IBC (Intra-Block Copy) não corresponde ao tamanho do bloco CTU, libera a memória.
  if (m_IBCBufferWidth != IBC_BUFFER_SIZE / ctuSize)
  {
    m_IBCBuffer.destroy();
  }

  // Se o buffer IBC não contém buffers, cria-os.
  if (m_IBCBuffer.bufs.empty())
  {
    m_IBCBufferWidth = IBC_BUFFER_SIZE / ctuSize;
    m_IBCBuffer.create(UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize)));
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion(const PredictionUnit& pu)
{
  const Slice& slice = *pu.cs->slice;

  // Verifica se é um bloco de predição de um slice de tipo InterB e a ponderação bipred não está ativada.
  if (slice.isInterB() && !pu.cs->pps->getWPBiPred())
  {
    // Verifica se ambas as referências estão válidas.
    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      // Obtém os quadros de referência para as listas de referência.
      const Picture* refPicL0 = slice.getRefPic(REF_PIC_LIST_0, pu.refIdx[0]);
      const Picture* refPicL1 = slice.getRefPic(REF_PIC_LIST_1, pu.refIdx[1]);

      // Verifica se ambos os blocos de predição estão referenciando o mesmo quadro.
      if (refPicL0 == refPicL1)
      {
        // Se não for um bloco de predição com afinação...
        if (!pu.cu->affine)
        {
          // Verifica se os vetores de movimento são idênticos.
          if (pu.mv[0] == pu.mv[1])
          {
            return true;  // Movimento idêntico.
          }
        }
        else
        {
          // Se for um bloco de predição com afinação...
          if ((pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1])) ||
              (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])))
          {
            return true;  // Movimento idêntico.
          }
        }
      }
    }
  }

  return false;  // Movimento não é idêntico.
}

void InterPrediction::xSubPuMC(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList& eRefPicList, const bool luma,
                                const bool chroma)
{
  // Computa a posição e o tamanho da PU atual.
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
    // Calcula o número de partições na linha e coluna, altura e largura da PU.
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  }

  //Declara uma variável subPu do tipo PredictionUnit para representar a sub-PU
  PredictionUnit subPu;

  // Inicializa a sub-PU com informações da PU original.
  subPu.cs = pu.cs;
  subPu.cu = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  // Salva e desativa a flag de afinação da PU original para evitar conflitos com as sub-PUs.
  bool isAffine = pu.cu->affine;
  subPu.cu->affine = false;

  // Verifica se a direção vertical (verMC) da PU é maior que a direção horizontal.
  bool verMC = puSize.height > puSize.width;
  // Inicializa variáveis para iterar sobre as dimensões da PU.
  int fstStart = (!verMC ? puPos.y : puPos.x);
  int secStart = (!verMC ? puPos.x : puPos.y);
  int fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int fstStep = (!verMC ? puHeight : puWidth);
  int secStep = (!verMC ? puWidth : puHeight);

  // Verifica se a PU está dimensionada.
  bool scaled = pu.cu->slice->getRefPic(REF_PIC_LIST_0, 0)->isRefScaled(pu.cs->pps) ||
                (pu.cs->slice->getSliceType() == B_SLICE ? pu.cu->slice->getRefPic(REF_PIC_LIST_1, 0)->isRefScaled(pu.cs->pps) : false);

  // Ativa a flag indicando que estamos processando sub-PUs.
  m_subPuMC = true;

  // Itera sobre as dimensões da PU.
  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo& curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later = secDim + secStep;

      // Agrupa sub-PUs com o mesmo vetor de movimento.
      while (later < secEnd)
      {
        const MotionInfo& laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (!scaled && laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      // Inicializa a sub-PU com a área da PU atual.
      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;

      // Obtém a sub-predição buffer para a sub-PU.
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      // Desativa temporariamente o modo MMVD e o refinamento de movimento para evitar interferência com a predição.
      subPu.mmvdEncOptMode = 0;
      subPu.mvRefine = false;

      // Realiza a compensação de movimento para a sub-PU.
      motionCompensation(subPu, subPredBuf, eRefPicList, luma, chroma, nullptr);

      secDim = later - secStep;  // Atualiza a posição para a próxima iteração.
    }
  }

  // Desativa a flag de processamento de sub-PUs.
  m_subPuMC = false;

  // Restaura a flag de afinação da PU original.
  pu.cu->affine = isAffine;
}


void InterPrediction::xSubPuBio(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList& eRefPicList, PelUnitBuf* yuvDstTmp)
{
  // Computa a posição e o tamanho da PU atual.
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  JVET_J0090_SET_CACHE_ENABLE(true);
  int mvShift = (MV_FRACTIONAL_BITS_INTERNAL);
  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);
    for (int compID = 0; compID < MAX_NUM_COMPONENT; compID++)
    {
      // Calcula a posição e o tamanho da referência para a banda de aplicação.
      Mv cMv = pu.mv[refId];
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
      int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
      cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTemp), -(((filtersize >> 1) - 1) << mvshiftTemp));
      bool wrapRef = false;
      if (pu.cu->slice->getRefPic(refId, pu.refIdx[refId])->isWrapAroundEnabled(pu.cs->pps))
      {
        wrapRef = wrapClipMv(cMv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps);
      }
      else
      {
        clipMv(cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
      }

      int width = predBuf.bufs[compID].width + (filtersize - 1);
      int height = predBuf.bufs[compID].height + (filtersize - 1);

      // Obtém o bloco de referência.
      CPelBuf refBuf;
      Position recOffset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTemp, cMv.getVer() >> mvshiftTemp);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, recOffset, pu.blocks[compID].size()), wrapRef);

      // Medição de acesso à cache para fins de análise de largura de banda.
      JVET_J0090_SET_REF_PICTURE(refPic, (ComponentID)compID);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          JVET_J0090_CACHE_ACCESS(((Pel*)refBuf.buf) + row * refBuf.stride + col, __FILE__, __LINE__);
        }
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(false);
#endif

  PredictionUnit subPu;

  // Inicializa a sub-PU com informações da PU original.
  subPu.cs = pu.cs;
  subPu.cu = pu.cu;
  subPu.mergeType = pu.mergeType;
  subPu.mmvdMergeFlag = pu.mmvdMergeFlag;
  subPu.mmvdEncOptMode = pu.mmvdEncOptMode;
  subPu.mergeFlag = pu.mergeFlag;
  subPu.ciipFlag = pu.ciipFlag;
  subPu.mvRefine = pu.mvRefine;
  subPu.refIdx[0] = pu.refIdx[0];
  subPu.refIdx[1] = pu.refIdx[1];

  // Inicializa variáveis para iterar sobre as dimensões da PU.
  int fstStart = puPos.y;
  int secStart = puPos.x;
  int fstEnd = puPos.y + puSize.height;
  int secEnd = puPos.x + puSize.width;
  int fstStep = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.height);
  int secStep = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.width);

  // Itera sobre as dimensões da PU.
  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = secDim;
      int y = fstDim;
      int dx = secStep;
      int dy = fstStep;

      // Obtém as informações de movimento da sub-PU atual.
      const MotionInfo& curMi = pu.getMotionInfo(Position{ x, y });

      // Inicializa a sub-PU com a área da PU atual.
      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;

      // Obtém o buffer de predição para a sub-PU.
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      // Realiza a compensação de movimento para a sub-PU, opcionalmente usando um buffer temporário de destino.
      if (yuvDstTmp)
      {
        PelUnitBuf subPredBufTmp = yuvDstTmp->subBuf(UnitAreaRelative(pu, subPu));
        motionCompensation(subPu, subPredBuf, eRefPicList, true, true, &subPredBufTmp);
      }
      else
      {
        motionCompensation(subPu, subPredBuf, eRefPicList);
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(true);
  /*
    A função processa sub-PUs em uma PU, realizando a compensação de movimento com suporte especial para bordas desfocadas (BDOF).
    As flags e informações da PU original são preservadas na sub-PU.
    Medição de acesso à cache (JVET_J0090_CACHE_ACCESS) é realizada para análise de largura de banda em cenários específicos (JVET_J0090_MEMORY_BANDWITH_MEASURE).
    A função pode usar um buffer temporário de destino (yuvDstTmp) para armazenar a predição da sub-PU.
    A função manipula temporariamente as flags mmvdMergeFlag, mmvdEncOptMode, mergeFlag, ciipFlag e mvRefine para evitar conflitos durante
  */
}

void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred,
                                    const bool bi, const bool bioApplied, const bool luma, const bool chroma)
{
  // Obtém referências relevantes.
  const SPS& sps = *pu.cs->sps;

  // Inicializa variáveis para as informações de movimento e identifica se a PU é IBC.
  int refIdx = pu.refIdx[eRefPicList];
  Mv mv[3];
  bool isIBC = false;

  // Verifica se a PU é IBC.
  CHECK(!CU::isIBC(*pu.cu) && pu.lwidth() == 4 && pu.lheight() == 4, "invalid 4x4 inter blocks");
  if (CU::isIBC(*pu.cu))
  {
    isIBC = true;
  }

  // Obtém os vetores de movimento.
  if (pu.cu->affine)
  {
    CHECK(refIdx < 0, "refIdx incorrect.");

    mv[0] = pu.mvAffi[eRefPicList][0];
    mv[1] = pu.mvAffi[eRefPicList][1];
    mv[2] = pu.mvAffi[eRefPicList][2];
  }
  else
  {
    mv[0] = pu.mv[eRefPicList];
  }

  // Ajusta os vetores de movimento, se necessário.
  if (!pu.cu->affine)
  {
    if (!isIBC && pu.cu->slice->getRefPic(eRefPicList, refIdx)->isRefScaled(pu.cs->pps) == false)
    {
      if (!pu.cs->pps->getWrapAroundEnabledFlag())
      {
        clipMv(mv[0], pu.cu->lumaPos(), pu.cu->lumaSize(), sps, *pu.cs->pps);
      }
    }
  }

  // Itera sobre os componentes (luma, croma) para realizar a predição.
  for (uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++)
  {
    const ComponentID compID = ComponentID(comp);

    // Verifica se a componente deve ser processada.
    if (compID == COMPONENT_Y && !luma)
    {
      continue;
    }
    if (compID != COMPONENT_Y && !chroma)
    {
      continue;
    }

    // Realiza a predição para a PU com base nas configurações.
    if (pu.cu->affine)
    {
      CHECK(bioApplied, "BIO is not allowed with affine");
      m_iRefListIdx = eRefPicList;
      bool genChromaMv = (!luma && chroma && compID == COMPONENT_Cb);
      xPredAffineBlk(compID, pu, pu.cu->slice->getRefPic(eRefPicList, refIdx)->unscaledPic, mv, pcYuvPred, bi,
                     pu.cu->slice->clpRng(compID), genChromaMv, pu.cu->slice->getScalingRatio(eRefPicList, refIdx));
    }
    else
    {
      if (isIBC)
      {
        xPredInterBlk(compID, pu, pu.cu->slice->getPic(), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng(compID),
                      bioApplied, isIBC);
      }
      else
      {
        xPredInterBlk(compID, pu, pu.cu->slice->getRefPic(eRefPicList, refIdx)->unscaledPic, mv[0], pcYuvPred, bi,
                      pu.cu->slice->clpRng(compID), bioApplied, isIBC,
                      pu.cu->slice->getScalingRatio(eRefPicList, refIdx), 0, 0, false, nullptr, 0);
      }
    }
  }
  /*
    A função trata a predição de uma unidade de predição (PU) em blocos de 4x4 para uma única lista de referência (luma ou crominância).
    As opções de predição incluem tanto predição uni-direcional quanto bidirecional.
    A função suporta predição affine e verifica se a PU é do tipo IBC.
    A predição é realizada para cada componente (luma, croma) com base nas configurações específicas para cada componente.
    Se a PU é affine, a predição é realizada pela função xPredAffineBlk, e se a PU é IBC, a predição é realizada pela função `xPredInterBlk
  */
}

/*################################################################################
 * \brief Realiza predição bidirecional para uma PredictionUnit específica.
 * Esta função realiza predição de movimento bidirecional para uma PredictionUnit específica.
 * \param pu           Referência para a PredictionUnit.
 * \param pcYuvPred    PelUnitBuf de saída para as amostras previstas.
 * \param luma         Flag indicando se a predição de luma é realizada.
 * \param chroma       Flag indicando se a predição de croma é realizada.
 * \param yuvPredTmp   PelUnitBuf temporário usado para resultados intermediários (opcional).
 */
void InterPrediction::xPredInterBi(PredictionUnit &pu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma,
                                   PelUnitBuf *yuvPredTmp)
{
  // Acessando informações de PPS e Slice
  const PPS &pps = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;

  // Recuperando índices de referência para as listas 0 e 1
  const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
  const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

  // Verificando a validade para certos tamanhos de bloco ao não usar movimento afim
  CHECK(!pu.cu->affine && refIdx0 >= 0 && refIdx1 >= 0 && pu.lwidth() + pu.lheight() == 12,
        "blocos bi-previstos 4x8/8x4 inválidos");

  // Inicializando flags para aplicação de bio e DMVR
  bool bioApplied = false;
  if (pu.cs->sps->getBDOFEnabledFlag() && !pu.cs->picHeader->getBdofDisabledFlag())
  {
    if (pu.cu->affine || m_subPuMC)
    {
      bioApplied = false;
    }
    else
    {
      bioApplied = PU::isSimpleSymmetricBiPred(pu) && PU::dmvrBdofSizeCheck(pu) && !pu.ciipFlag && !pu.cu->smvdMode;
    }
  }

  // Verificando condições para desativar o filtro bio com base no modo de otimização MMVD
  if (pu.mmvdEncOptMode == 2 && pu.mmvdMergeFlag)
  {
    bioApplied = false;
  }

  // Verificando condições para a aplicação de DMVR
  bool dmvrApplied = (pu.mvRefine) && PU::checkDMVRCondition(pu);

  // Verificando se as imagens de referência estão escaladas
  bool refIsScaled = (refIdx0 < 0 ? false : pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0)->isRefScaled(pu.cs->pps)) ||
                     (refIdx1 < 0 ? false : pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1)->isRefScaled(pu.cs->pps));
  dmvrApplied = dmvrApplied && !refIsScaled;
  bioApplied = bioApplied && !refIsScaled;

  // Iterando pelas listas de imagens de referência para predição
  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if (pu.refIdx[refList] < 0)
    {
      continue;
    }

    // Determinando a lista de imagens de referência (0 ou 1)
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    // Verificando a validade para o modo IBC e índices de referência
    CHECK(CU::isIBC(*pu.cu) && eRefPicList != REF_PIC_LIST_0, "Direção interdir inválida para modo ibc");
    CHECK(CU::isIBC(*pu.cu) && pu.refIdx[refList] != MAX_NUM_REF, "Índice de referência inválido para modo ibc");
    CHECK((CU::isInter(*pu.cu) && pu.refIdx[refList] >= slice.getNumRefIdx(eRefPicList)), "Índice de referência inválido");
    m_iRefListIdx = refList;

    // Criando um PelUnitBuf para amostras da imagem de referência
    PelUnitBuf pcMbBuf = (pu.chromaFormat == CHROMA_400
                              ? PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()))
                              : PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()),
                                           PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()),
                                           PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())));

    // Verificando condições para predição uni ou bi-direcional
    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      // Verificando se DMVR está sendo aplicado e usando um buffer temporário para resultados intermediários
      if (dmvrApplied)
      {
        if (yuvPredTmp)
        {
          xPredInterUni(pu, eRefPicList, pcMbBuf, true, false, luma, chroma);
        }
        continue;
      }
      // Realizando predição de movimento
      xPredInterUni(pu, eRefPicList, pcMbBuf, true, bioApplied, luma, chroma);
    }
    else
    {
      // Verificando condições para predição ponderada
      if (((pps.getUseWP() && slice.getSliceType() == P_SLICE) ||
           (pps.getWPBiPred() && slice.getSliceType() == B_SLICE)))
      {
        // Realizando predição de movimento
        xPredInterUni(pu, eRefPicList, pcMbBuf, true, bioApplied, luma, chroma);
      }
      else
      {
        // Realizando predição de movimento com transformação geométrica
        xPredInterUni(pu, eRefPicList, pcMbBuf, pu.cu->geoFlag, bioApplied, luma, chroma);
      }
    }
  }

  // Criando CPelUnitBuf para previsões de origem
  CPelUnitBuf srcPred0 = (pu.chromaFormat == CHROMA_400
                              ? CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()))
                              : CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()),
                                            PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()),
                                            PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())));
  CPelUnitBuf srcPred1 = (pu.chromaFormat == CHROMA_400
                              ? CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()))
                              : CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()),
                                            PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()),
                                            PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())));

  // Verificando condições para predição exclusiva de luma ou croma
  const bool lumaOnly = luma && !chroma;
  const bool chromaOnly = !luma && chroma;

  // Verificando condições para predição ponderada bidirecional, predição ponderada unidirecional ou predição padrão
  if (!pu.cu->geoFlag && (!dmvrApplied) && (!bioApplied) && pps.getWPBiPred() && slice.getSliceType() == B_SLICE &&
      pu.cu->bcwIdx == BCW_DEFAULT)
  {
    // Realizando predição ponderada bidirecional
    xWeightedPredictionBi(pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred, lumaOnly, chromaOnly);

    // Copiando resultados para o buffer temporário, se fornecido
    if (yuvPredTmp)
    {
      yuvPredTmp->copyFrom(pcYuvPred);
    }
  }
  else if (!pu.cu->geoFlag && pps.getUseWP() && slice.getSliceType() == P_SLICE)
  {
    // Realizando predição ponderada unidirecional para slices P
    xWeightedPredictionUni(pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred, lumaOnly, chromaOnly);

    // Copiando resultados para o buffer temporário, se fornecido
    if (yuvPredTmp)
    {
      yuvPredTmp->copyFrom(pcYuvPred);
    }
  }
  else
  {
    // Verificando condições para a aplicação de DMVR
    if (dmvrApplied)
    {
      // Adicionando a média das previsões de origem e processando DMVR
      if (yuvPredTmp)
      {
        yuvPredTmp->addAvg(srcPred0, srcPred1, slice.clpRngs(), false);
      }
      xProcessDMVR(pu, pcYuvPred, slice.clpRngs(), bioApplied);
    }
    else
    {
      // Realizando média ponderada ou predição padrão
      xWeightedAverage(pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), bioApplied,
                       lumaOnly, chromaOnly, yuvPredTmp);
    }
  }
}

/*################################################################################
 * \brief Realiza a predição de um bloco inter utilizando diferentes métodos de interpolação.
 * Esta função realiza a predição de um bloco inter para um componente específico (luma ou chroma)
 * utilizando diferentes métodos de interpolação, incluindo RPR (Reconstruction Precision Refinement),
 * filtragem bilinear e outros.
 * \param compID         Identificador do componente para o qual a predição é realizada (luma ou chroma).
 * \param pu             Referência para a PredictionUnit associada ao bloco inter.
 * \param refPic         Ponteiro para a Picture de referência.
 * \param _mv            Vetor de movimento que especifica o deslocamento em relação à posição de referência.
 * \param dstPic         PelUnitBuf de saída para as amostras previstas.
 * \param bi             Flag indicando se a predição é bidirecional.
 * \param clpRng         Faixa de valores permitidos após a aplicação do processo de clippagem.
 * \param bioApplied     Flag indicando se o filtro BIO (Blind Optical Flow) foi aplicado.
 * \param isIBC          Flag indicando se a predição é para um bloco IBC (Inter-Block Copy).
 * \param scalingRatio   Par de fatores de escala horizontal e vertical para RPR (opcional).
 * \param dmvrWidth      Largura do bloco utilizado em DMVR (opcional).
 * \param dmvrHeight     Altura do bloco utilizado em DMVR (opcional).
 * \param bilinearMC     Flag indicando se a filtragem bilinear é utilizada em MC (Motion Compensation).
 * \param srcPadBuf      Ponteiro para o buffer de preenchimento de borda (opcional).
 * \param srcPadStride   Passo de stride para o buffer de preenchimento de borda.
 */
void InterPrediction::xPredInterBlk(const ComponentID compID, const PredictionUnit &pu, const Picture *refPic,
                                    const Mv &_mv, PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng,
                                    const bool bioApplied, bool isIBC, const std::pair<int, int> scalingRatio,
                                    SizeType dmvrWidth, SizeType dmvrHeight, bool bilinearMC, Pel *srcPadBuf,
                                    int32_t srcPadStride)
{
  // Configurando a Picture de referência
  JVET_J0090_SET_REF_PICTURE(refPic, compID);

  // Obtendo informações sobre o formato cromático e se a operação é de arredondamento
  const ChromaFormat chFmt = pu.chromaFormat;
  const bool rndRes = !bi;

  // Configurando o deslocamento para interpolação
  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compID, chFmt);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compID, chFmt);

  // Verificando se é necessário aplicar a interpolação para uma referência "embrulhada" (wrap-around)
  bool wrapRef = false;
  Mv mv(_mv);
  if (!isIBC && refPic->isWrapAroundEnabled(pu.cs->pps))
  {
    wrapRef = wrapClipMv(mv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps);
  }

  // Determinando se usar o filtro alternativo (HPEL) para interpolação
  bool useAltHpelIf = pu.cu->imv == IMV_HPEL;

  // Verificando se a predição de bloco RPR foi aplicada com sucesso
  if (!isIBC &&
      xPredInterBlkRPR(scalingRatio, *pu.cs->pps, CompArea(compID, chFmt, pu.blocks[compID], dstPic.bufs[compID]),
                       refPic, mv, dstPic.bufs[compID].buf, dstPic.bufs[compID].stride, bi, wrapRef, clpRng,
                       InterpolationFilter::Filter::DEFAULT, useAltHpelIf))
  {
    // Validando as configurações quando RPR é aplicado
    CHECK(bilinearMC, "DMVR deve ser desativado com RPR");
    CHECK(bioApplied, "BDOF deve ser desativado com RPR");
  }
  else
  {
    // Configurando as frações horizontais e verticais para interpolação
    int xFrac, yFrac;
    if (isIBC)
    {
      xFrac = yFrac = 0;
      JVET_J0090_SET_CACHE_ENABLE(false);
    }
    else if (isLuma(compID))
    {
      xFrac = mv.hor & 15;
      yFrac = mv.ver & 15;
    }
    else
    {
      xFrac = mv.hor * (1 << (1 - ::getComponentScaleX(compID, chFmt))) & 31;
      yFrac = mv.ver * (1 << (1 - ::getComponentScaleY(compID, chFmt))) & 31;
    }

    // Obtendo referência para o buffer de destino
    PelBuf &dstBuf = dstPic.bufs[compID];
    unsigned width = dstBuf.width;
    unsigned height = dstBuf.height;

    // Configurando buffer de referência
    CPelBuf refBuf;
    {
      Position offset = pu.blocks[compID].pos().offset(mv.getHor() >> shiftHor, mv.getVer() >> shiftVer);
      if (dmvrWidth)
      {
        refBuf = refPic->getRecoBuf(CompArea(compID, chFmt, offset, Size(dmvrWidth, dmvrHeight)), wrapRef);
      }
      else
      {
        refBuf = refPic->getRecoBuf(CompArea(compID, chFmt, offset, pu.blocks[compID].size()), wrapRef);
      }
    }

    // Configurando o buffer de referência para preenchimento de borda, se fornecido
    if (nullptr != srcPadBuf)
    {
      refBuf.buf = srcPadBuf;
      refBuf.stride = srcPadStride;
    }

    // Ajustando a largura e altura se DMVR estiver ativado
    if (dmvrWidth)
    {
      width = dmvrWidth;
      height = dmvrHeight;
    }

    // Backup dos dados
    int backupWidth = width;
    int backupHeight = height;
    Pel *backupDstBufPtr = dstBuf.buf;
    int backupDstBufStride = dstBuf.stride;

    // Configurando buffer temporário para filtragem BIO
    if (bioApplied && compID == COMPONENT_Y)
    {
      width = width + 2 * BIO_EXTEND_SIZE + 2;
      height = height + 2 * BIO_EXTEND_SIZE + 2;

      // Alterando a saída da MC
      dstBuf.stride = width;
      dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 2;
    }

    // Determinando o filtro a ser utilizado (RPR, HPEL, padrão)
    const auto filterIdx =
        bilinearMC ? InterpolationFilter::Filter::DMVR
                   : (useAltHpelIf ? InterpolationFilter::Filter::HALFPEL_ALT : InterpolationFilter::Filter::DEFAULT);

    // Realizando a interpolação horizontal, vertical ou ambas, conforme necessário
    if (yFrac == 0)
    {
      m_if.filterHor(compID, (Pel *)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight,
                     xFrac, rndRes, clpRng, filterIdx);
    }
    else if (xFrac == 0)
    {
      m_if.filterVer(compID, (Pel *)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight,
                     yFrac, true, rndRes, clpRng, filterIdx);
    }
    else
    {
      // Configurando o buffer temporário para interpolação bilinear
      PelBuf tmpBuf = dmvrWidth ? PelBuf(m_filteredBlockTmp[0][compID], Size(dmvrWidth, dmvrHeight))
                                : PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
      if (dmvrWidth == 0)
      {
        tmpBuf.stride = dstBuf.stride;
      }

      int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
      if (bilinearMC)
      {
        vFilterSize = NTAPS_BILINEAR;
      }
      m_if.filterHor(compID, (Pel *)refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf,
                     tmpBuf.stride, backupWidth, backupHeight + vFilterSize - 1, xFrac, false, clpRng, filterIdx);
      JVET_J0090_SET_CACHE_ENABLE(false);
      m_if.filterVer(compID, (Pel *)tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf,
                     dstBuf.stride, backupWidth, backupHeight, yFrac, false, rndRes, clpRng, filterIdx);
    }
    JVET_J0090_SET_CACHE_ENABLE((srcPadStride == 0) &&
                                (bioApplied == false)); // Habilitado apenas no processo não-DMVR-não-BDOF, em processos DMVR, srcPadStride é sempre não-zero

    // Configurando buffer temporário para filtragem BIO, se aplicada
    if (bioApplied && compID == COMPONENT_Y)
    {
      const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
      int xOffset = (xFrac < 8) ? 1 : 0;
      int yOffset = (yFrac < 8) ? 1 : 0;
      const Pel *refPel = refBuf.buf - yOffset * refBuf.stride - xOffset;
      Pel *dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + dstBuf.stride + 1;
      for (int w = 0; w < (width - 2 * BIO_EXTEND_SIZE); w++)
      {
        Pel val = leftShift_round(refPel[w], shift);
        dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
      }

      refPel = refBuf.buf + (1 - yOffset) * refBuf.stride - xOffset;
      dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 1;
      for (int h = 0; h < (height - 2 * BIO_EXTEND_SIZE - 2); h++)
      {
        Pel val = leftShift_round(refPel[0], shift);
        dstPel[0] = val - (Pel)IF_INTERNAL_OFFS;

        val = leftShift_round(refPel[width - 3], shift);
        dstPel[width - 3] = val - (Pel)IF_INTERNAL_OFFS;

        refPel += refBuf.stride;
        dstPel += dstBuf.stride;
      }

      refPel = refBuf.buf + (height - 2 * BIO_EXTEND_SIZE - 2 + 1 - yOffset) * refBuf.stride - xOffset;
      dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + (height - 2 * BIO_EXTEND_SIZE) * dstBuf.stride + 1;
      for (int w = 0; w < (width - 2 * BIO_EXTEND_SIZE); w++)
      {
        Pel val = leftShift_round(refPel[w], shift);
        dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
      }

      // Restaurando os dados
      width = backupWidth;
      height = backupHeight;
      dstBuf.buf = backupDstBufPtr;
      dstBuf.stride = backupDstBufStride;
    }
  }
}

/**
 * \brief Verifica se o vetor do subbloco se espalha além do limite especificado.
 *
 * Esta função verifica se o vetor de um subbloco se espalha além de um limite especificado.
 *
 * \param a           Coordenada x do ponto A do bloco.
 * \param b           Coordenada y do ponto A do bloco.
 * \param c           Coordenada x do ponto B do bloco.
 * \param d           Coordenada y do ponto B do bloco.
 * \param predType    Tipo de predição.
 *                      - 0: Intra
 *                      - 1: Inter (não-BiPred)
 *                      - 2: BiPred (Predição Bidirecional)
 *                      - 3: BiPred (Predição Bidirecional com MVs diferentes)
 *
 * \return true se o vetor do subbloco se espalha além do limite, false caso contrário.
 */
bool InterPrediction::isSubblockVectorSpreadOverLimit(int a, int b, int c, int d, int predType)
{
  // Constantes relacionadas ao filtro
  int s4 = (4 << 11); //pq 8192? (4x2^11)
  int filterTap = 6;

  if (predType == 3)
  {
    // Calculando a largura e altura do bloco de referência
    int refBlkWidth = std::max(std::max(0, 4 * a + s4), std::max(4 * c, 4 * a + 4 * c + s4)) - std::min(std::min(0, 4 * a + s4), std::min(4 * c, 4 * a + 4 * c + s4));
    int refBlkHeight = std::max(std::max(0, 4 * b), std::max(4 * d + s4, 4 * b + 4 * d + s4)) - std::min(std::min(0, 4 * b), std::min(4 * d + s4, 4 * b + 4 * d + s4));

    // Convertendo as dimensões para unidades de filtro e adicionando margens
    refBlkWidth = (refBlkWidth >> 11) + filterTap + 3;
    refBlkHeight = (refBlkHeight >> 11) + filterTap + 3;

    // Verificando se o bloco de referência excede o tamanho limite
    if (refBlkWidth * refBlkHeight > (filterTap + 9) * (filterTap + 9))
    {
      return true;
    }
  }
  else
  {
    // Calculando a largura e altura do bloco de referência para o primeiro conjunto de pontos
    int refBlkWidth = std::max(0, 4 * a + s4) - std::min(0, 4 * a + s4);
    int refBlkHeight = std::max(0, 4 * b) - std::min(0, 4 * b);

    // Convertendo as dimensões para unidades de filtro e adicionando margens
    refBlkWidth = (refBlkWidth >> 11) + filterTap + 3;
    refBlkHeight = (refBlkHeight >> 11) + filterTap + 3;

    // Verificando se o bloco de referência excede o tamanho limite
    if (refBlkWidth * refBlkHeight > (filterTap + 9) * (filterTap + 5))
    {
      return true;
    }

    // Calculando a largura e altura do bloco de referência para o segundo conjunto de pontos
    refBlkWidth = std::max(0, 4 * c) - std::min(0, 4 * c);
    refBlkHeight = std::max(0, 4 * d + s4) - std::min(0, 4 * d + s4);

    // Convertendo as dimensões para unidades de filtro e adicionando margens
    refBlkWidth = (refBlkWidth >> 11) + filterTap + 3;
    refBlkHeight = (refBlkHeight >> 11) + filterTap + 3;

    // Verificando se o bloco de referência excede o tamanho limite
    if (refBlkWidth * refBlkHeight > (filterTap + 5) * (filterTap + 9))
    {
      return true;
    }
  }
  return false;
}

#if GDR_ENABLED
/**
 * \brief GDR (Gradient-Domain Residual) habilitado: Realiza a predição de bloco com afinidade.
 *
 * Esta função realiza a predição de um bloco com afinidade quando o GDR (Gradient-Domain Residual) está habilitado.
 *
 * \param compID       Identificador do componente de cor (luma ou crominância).
 * \param pu           Unidade de predição.
 * \param refPic       Imagem de referência.
 * \param _mv          Vetor de movimento.
 * \param dstPic       Buffer de imagem de destino.
 * \param bi           Flag indicando se a predição é bidirecional.
 * \param clpRng       Faixa de valores permitida após a predição.
 * \param genChromaMv  Flag indicando se a predição de crominância deve gerar vetores de movimento distintos.
 * \param scalingRatio Razão de escalonamento para ajustar o tamanho do bloco.
 *
 * \return true se a predição for bem-sucedida, false caso contrário.
 */
bool InterPrediction::xPredAffineBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic,
                                     const Mv *_mv, PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng,
                                     bool genChromaMv, const std::pair<int, int> scalingRatio)
#else
/**
 * \brief GDR (Gradient-Domain Residual) desabilitado: Realiza a predição de bloco com afinidade.
 *
 * Esta função realiza a predição de um bloco com afinidade quando o GDR (Gradient-Domain Residual) está desabilitado.
 *
 * \param compID       Identificador do componente de cor (luma ou crominância).
 * \param pu           Unidade de predição.
 * \param refPic       Imagem de referência.
 * \param _mv          Vetor de movimento.
 * \param dstPic       Buffer de imagem de destino.
 * \param bi           Flag indicando se a predição é bidirecional.
 * \param clpRng       Faixa de valores permitida após a predição.
 * \param genChromaMv  Flag indicando se a predição de crominância deve gerar vetores de movimento distintos.
 * \param scalingRatio Razão de escalonamento para ajustar o tamanho do bloco.
 */
void InterPrediction::xPredAffineBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic,
                                     const Mv *_mv, PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng,
                                     bool genChromaMv, const std::pair<int, int> scalingRatio)
#endif
{
// Define a imagem de referência para a componente especificada (Luma, Croma)
JVET_J0090_SET_REF_PICTURE(refPic, compID);

// Obtém informações sobre a crominância (formato de croma) e a estrutura de codificação
const ChromaFormat &chFmt = pu.chromaFormat;
const CodingStructure &cs  = *pu.cs;
const SPS &            sps = *cs.sps;
const PPS &            pps = *cs.pps;
const PicHeader &      ph  = *cs.picHeader;

#if GDR_ENABLED
// Inicializa uma variável para verificar se todos os blocos estão limpos durante a codificação GDR
bool allOk = true;
// Verifica se a codificação GDR está habilitada e se o bloco está limpo
const bool isEncodeGdrClean = sps.getGDREnabledFlag() && cs.pcv->isEncoder &&
  ((ph.getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA)) ||
   ph.getNumVerVirtualBoundaries() == 0);
// Obtém as coordenadas x e y do bloco
const int pux = pu.lx();
const int puy = pu.ly();
#endif

// Obtém as dimensões do bloco de predição (luma)
const int widthLuma  = pu.Y().width;
const int heightLuma = pu.Y().height;

// Define o tamanho dos subblocos afins
const int sbWidth  = AFFINE_SUBBLOCK_SIZE;
const int sbHeight = AFFINE_SUBBLOCK_SIZE;

// Verifica se a referência está dimensionada
const bool isRefScaled = refPic->isRefScaled(&pps);

// Calcula as dimensões extendidas para a saída e buffers temporários
const int dstExtW = (sbWidth + PROF_BORDER_EXT_W * 2 + 7) & ~7;
const int dstExtH = sbHeight + PROF_BORDER_EXT_H * 2;
PelBuf dstExtBuf(m_filteredBlockTmp[1][compID], dstExtW, dstExtH);

const int refExtH = dstExtH + MAX_FILTER_SIZE - 1;
PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], dstExtW, refExtH);

// Obtém o buffer de destino para a componente específica
PelBuf &dstBuf = dstPic.bufs[compID];

// Verifica se a referência suporta a rotação (wrapAround)
const bool wrapAroundEnabled = refPic->isWrapAroundEnabled(&pps);

// Inicializa variáveis de vetor de movimento diferenciais
int dmvHorX = 0;
int dmvHorY = 0;
int dmvVerX = 0;
int dmvVerY = 0;

// Precisão para cálculos com base na profundidade máxima do bloco
constexpr int PREC = MAX_CU_DEPTH;

// Verifica se a Profiling está habilitada para a componente Y e não está desativada na imagem
bool enableProfTmp = compID == COMPONENT_Y && !ph.getProfDisabledFlag() && !isRefScaled && !m_skipProf;

// Verifica se a componente é Y, ou se os vetores de movimento de croma estão sendo gerados, ou se o formato de croma é 4:4:4
if (compID == COMPONENT_Y || genChromaMv || chFmt == CHROMA_444)
{
  // Obtém os vetores de movimento dos cantos do bloco de predição afim
  const Mv &mvLT = _mv[0];
  const Mv &mvRT = _mv[1];
  const Mv &mvLB = _mv[2];

  // Calcula os vetores de movimento diferenciais horizontal e vertical baseados nos cantos do bloco
  dmvHorX = (mvRT - mvLT).getHor() * (1 << (PREC - floorLog2(widthLuma)));
  dmvHorY = (mvRT - mvLT).getVer() * (1 << (PREC - floorLog2(widthLuma)));
  if (pu.cu->affineType == AFFINEMODEL_6PARAM)
  {
    dmvVerX = (mvLB - mvLT).getHor() * (1 << (PREC - floorLog2(heightLuma)));
    dmvVerY = (mvLB - mvLT).getVer() * (1 << (PREC - floorLog2(heightLuma)));
  }
  else
  {
    // Se o bloco não usa modelo afim de 6 parâmetros, calcula os vetores de movimento verticais baseados nos horizontais
    dmvVerX = -dmvHorY;
    dmvVerY = dmvHorX;
  }
}

// Verifica se todos os vetores de movimento são zero
if (dmvHorX == 0 && dmvHorY == 0 && dmvVerX == 0 && dmvVerY == 0)
{
  // Desativa o Profiling temporário se todos os vetores de movimento forem zero
  enableProfTmp = false;
}

// Verifica se o gradiente do vetor de movimento é grande (provavelmente um movimento rápido)
const bool largeMvGradient = isSubblockVectorSpreadOverLimit(dmvHorX, dmvHorY, dmvVerX, dmvVerY, pu.interDir);
if (largeMvGradient)
{
  // Desativa o Profiling temporário se o gradiente do vetor de movimento for grande
  enableProfTmp = false;
}

// Obtém as coordenadas base dos vetores de movimento
const int baseHor = mvLT.getHor() * (1 << PREC);
const int baseVer = mvLT.getVer() * (1 << PREC);

// Loop para preencher o buffer de vetores de movimento armazenados (MvBuffer)
for (int h = 0; h < heightLuma; h += sbHeight)
{
  for (int w = 0; w < widthLuma; w += sbWidth)
  {
    // Calcula os pesos para os vetores de movimento
    const int weightHor = largeMvGradient ? widthLuma >> 1 : (sbWidth >> 1) + w;
    const int weightVer = largeMvGradient ? heightLuma >> 1 : (sbHeight >> 1) + h;

    // Calcula o vetor de movimento temporário
    Mv tmpMv;
    tmpMv.hor = baseHor + dmvHorX * weightHor + dmvVerX * weightVer;
    tmpMv.ver = baseVer + dmvHorY * weightHor + dmvVerY * weightVer;

    // Arredonda e ajusta o vetor de movimento
    tmpMv.roundAffine(PREC - 4 + MV_FRACTIONAL_BITS_INTERNAL);
    tmpMv.clipToStorageBitDepth();

    // Armazena o vetor de movimento calculado no buffer
    m_storedMv[h / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_SUBBLOCK_SIZE] = tmpMv;
  }
}

// Verifica se o Profiling temporário está habilitado e a condição de skipProf é atendida
if (enableProfTmp && m_skipProfCond)
{
  // Limiar para determinar se o ajuste do vetor de movimento é pequeno o suficiente para pular o Profiling
  const int profThres = (m_biPredSearchAffine ? 2 : 1) << PREC;

  // Verifica se os ajustes dos vetores de movimento são pequenos o suficiente para pular o Profiling
  if (abs(dmvHorX) <= profThres && abs(dmvHorY) <= profThres && abs(dmvVerX) <= profThres && abs(dmvVerY) <= profThres)
  {
    // Desativa o Profiling temporário se os ajustes dos vetores de movimento forem pequenos
    enableProfTmp = false;
  }
}
// Determina se o Profiling está habilitado
const bool enableProf = enableProfTmp;

// Determina se é o último bloco e não é bidirecional
const bool isLast = !enableProf && !bi;

// Arrays para armazenar os valores escalados dos vetores de movimento
int dMvScaleHor[AFFINE_SUBBLOCK_SIZE * AFFINE_SUBBLOCK_SIZE];
int dMvScaleVer[AFFINE_SUBBLOCK_SIZE * AFFINE_SUBBLOCK_SIZE];

// Verifica se o Profiling está habilitado para realizar operações adicionais
if (enableProf)
{
  // Loop para calcular os valores escalados dos vetores de movimento
  for (int y = 0; y < sbHeight; y++)
  {
    for (int x = 0; x < sbWidth; x++)
    {
      const int wx = 2 * x - (sbWidth - 1);
      const int wy = 2 * y - (sbHeight - 1);

      // Calcula e armazena os valores escalados dos vetores de movimento
      dMvScaleHor[y * sbWidth + x] = wx * dmvHorX + wy * dmvVerX;
      dMvScaleVer[y * sbWidth + x] = wx * dmvHorY + wy * dmvVerY;
    }
  }
  
  // NOTE: the shift value is 7 and not 8 as in section 8.5.5.9 of the spec because
  // the values dMvScaleHor/dMvScaleVer are half of diffMvLX (which are always even
  // in equations 876 and 877)
  // Define a quantidade de bits para o deslocamento dos valores dos vetores de movimento
  const int mvShift = 7;
  // Limite para os vetores de movimento escalados
  const int dmvLimit = (1 << 5) - 1;

  // Tamanho dos arrays
  const int sz = sbWidth * sbHeight;

  // Verifica se a operação de arredondamento de vetores inteiros está disponível
  if (!g_pelBufOP.roundIntVector)
  {
    // Loop para arredondar e limitar os vetores de movimento escalados
    for (int idx = 0; idx < sz; idx++)
    {
      Mv tmpMv(dMvScaleHor[idx], dMvScaleVer[idx]);
      tmpMv.roundAffine(mvShift);
      dMvScaleHor[idx] = Clip3(-dmvLimit, dmvLimit, tmpMv.getHor());
      dMvScaleVer[idx] = Clip3(-dmvLimit, dmvLimit, tmpMv.getVer());
    }
  }
  else
  {
    // Utiliza a operação de arredondamento de vetores inteiros disponível
    g_pelBufOP.roundIntVector(dMvScaleHor, sz, mvShift, dmvLimit);
    g_pelBufOP.roundIntVector(dMvScaleVer, sz, mvShift, dmvLimit);
  }
}

// get prediction block by block
const int scaleX = ::getComponentScaleX(compID, chFmt);
const int scaleY = ::getComponentScaleY(compID, chFmt);

// Calcula as dimensões do bloco de predição considerando as escalas de componente
const int width  = widthLuma >> scaleX;
const int height = heightLuma >> scaleY;

CHECK(sbWidth > width, "Subblock width > block width");
CHECK(sbHeight > height, "Subblock height > block height");

// Itera sobre os blocos de predição
for (int h = 0; h < height; h += sbHeight)
{
  for (int w = 0; w < width; w += sbWidth)
  {
    Mv curMv;

    // Calcula as posições no domínio da luminância
    const int hLuma = h << scaleY;
    const int wLuma = w << scaleX;

    // Calcula o índice no buffer de MVs considerando os blocos de subamostragem
    const ptrdiff_t idx = hLuma / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + wLuma / AFFINE_SUBBLOCK_SIZE;

    // Obtém o vetor de movimento para o componente corrente
    if (compID == COMPONENT_Y || chFmt == CHROMA_444)
    {
      curMv = m_storedMv[idx];
    }
    else
    {
      curMv = m_storedMv[idx] + m_storedMv[idx + scaleY * MVBUFFER_SIZE + scaleX];
      curMv.roundAffine(1);
    }

    bool wrapRef = false;

    // Verifica se é necessário aplicar wrapping (envolvimento) ao vetor de movimento
    if (wrapAroundEnabled)
    {
      wrapRef = wrapClipMv(curMv, Position(pu.Y().x + wLuma, pu.Y().y + hLuma),
                           Size(sbWidth << scaleX, sbHeight << scaleY), &sps, &pps);
    }
    // Caso contrário, apenas realiza o clip do vetor de movimento
    else if (!isRefScaled)
    {
      clipMv(curMv, pu.lumaPos(), pu.lumaSize(), sps, pps);
    }
  }
}

#if GDR_ENABLED
if (isEncodeGdrClean)
{
  // Calcula a posição do sub-PU no domínio da luminância
  const Position subPuPos = Position(pux + ((w + sbWidth) << scaleX), puy + ((h + sbHeight) << scaleY));

  // Verifica se o sub-PU é considerado limpo (clean) com base na posição, vetor de movimento e referência
  const bool puClean = cs.isClean(subPuPos, curMv, refPic);

  // Atualiza a variável "allOk" com o resultado da verificação de limpeza do sub-PU
  allOk = allOk && puClean;
}
#endif
      // Seleciona o método de interpolação AFFINE
      const auto filterIdx = InterpolationFilter::Filter::AFFINE;
// Verifica se a referência está escalada
if (isRefScaled)
{
  // Verifica se PROF está desabilitado ao utilizar RPR (Reference Picture Resampling)
  CHECK(enableProf, "PROF should be disabled with RPR");

  // Realiza a predição para blocos AFFINE com RPR (Reference Picture Resampling)
  xPredInterBlkRPR(scalingRatio, pps,
                   CompArea(compID, chFmt, pu.blocks[compID].offset(w, h), Size(sbWidth, sbHeight)), refPic,
                   curMv, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, bi, wrapRef, clpRng, filterIdx);
}
else
{
  // Obtém o vetor de movimento em alta precisão
  int xFrac, yFrac, xInt, yInt;

  // Calcula coordenadas inteiras e fracionárias do vetor de movimento dependendo do componente de cor (luma ou croma)
  if (isLuma(compID))
  {
    xInt  = curMv.getHor() >> MV_FRAC_BITS_LUMA;
    xFrac = curMv.getHor() & MV_FRAC_MASK_LUMA;
    yInt  = curMv.getVer() >> MV_FRAC_BITS_LUMA;
    yFrac = curMv.getVer() & MV_FRAC_MASK_LUMA;
  }
  else
  {
    xInt  = curMv.getHor() * (1 << (1 - scaleX)) >> MV_FRAC_BITS_CHROMA;
    xFrac = curMv.getHor() * (1 << (1 - scaleX)) & MV_FRAC_MASK_CHROMA;
    yInt  = curMv.getVer() * (1 << (1 - scaleY)) >> MV_FRAC_BITS_CHROMA;
    yFrac = curMv.getVer() * (1 << (1 - scaleY)) & MV_FRAC_MASK_CHROMA;
  }
}

// Obtém o bloco de referência para predição
const CPelBuf refBuf = refPic->getRecoBuf(
  CompArea(compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID]), wrapRef);

// Pega ponteiro para o bloco de referência e a largura da linha
const Pel *ref = refBuf.buf;
const int refStride = refBuf.stride;

// Determina o destino para o resultado da predição
Pel *dst = enableProf ? dstExtBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H) : dstBuf.buf + w + h * dstBuf.stride;
const int dstStride = enableProf ? dstExtBuf.stride : dstBuf.stride;

// Verifica se a predição é apenas na direção horizontal (xFrac = 0)
if (yFrac == 0)
{
  // Realiza a predição horizontal
  m_if.filterHor(compID, ref, refStride, dst, dstStride, sbWidth, sbHeight, xFrac, isLast, clpRng, filterIdx);
}
// Verifica se a predição é apenas na direção vertical (yFrac = 0)
else if (xFrac == 0)
{
  // Realiza a predição vertical
  m_if.filterVer(compID, ref, refStride, dst, dstStride, sbWidth, sbHeight, yFrac, true, isLast, clpRng, filterIdx);
}
else
{
  // Caso de predição em ambas as direções (horizontal e vertical)
  // Calcula o tamanho do filtro a ser aplicado
  const int filterSize = isLuma(compID) ? NTAPS_LUMA_AFFINE : NTAPS_CHROMA_AFFINE;
  const int rowsAbove = (filterSize - 1) >> 1;

  // Aplica filtro horizontal na área estendida
  m_if.filterHor(compID, ref - rowsAbove * refStride, refStride, tmpBuf.buf, tmpBuf.stride, sbWidth,
                 sbHeight + filterSize - 1, xFrac, false, clpRng, filterIdx);
  // Desabilita a cache antes de aplicar o filtro vertical
  JVET_J0090_SET_CACHE_ENABLE(false);
  
  // Aplica filtro vertical na área estendida
  m_if.filterVer(compID, tmpBuf.buf + rowsAbove * tmpBuf.stride, tmpBuf.stride, dst, dstStride, sbWidth,
                 sbHeight, yFrac, false, isLast, clpRng, filterIdx);  
  // Reabilita a cache após a aplicação do filtro vertical
  JVET_J0090_SET_CACHE_ENABLE(true);
}

        //----------------------------------------------------------------
        // Verifica se o PROF está habilitado
        // PROF (Perceptually Optimized Filter for Redundancy)
        if (enableProf)
        {
          // Calcula o deslocamento com base na profundidade de bits do vídeo
          const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
          CHECKD(shift < 0, "shift must be positive");
          // Calcula as coordenadas inteiras para o deslocamento fracionário
          const int xOffset = xFrac >> (MV_FRAC_BITS_LUMA - 1);
          const int yOffset = yFrac >> (MV_FRAC_BITS_LUMA - 1);

          // NOTE: corners don't need to be padded
          // Inicializa ponteiros para referência e destino
          const Pel *refPel = ref + yOffset * refStride + xOffset;
          Pel *      dstPel = dst;

          // Preenche as bordas do bloco, aplicando o deslocamento e compensação
          for (ptrdiff_t x = 0; x < sbWidth; x++)
          {
            const ptrdiff_t refOffset = sbHeight * refStride;
            const ptrdiff_t dstOffset = sbHeight * dstStride;

            dstPel[x - dstStride] = (refPel[x - refStride] << shift) - IF_INTERNAL_OFFS;
            dstPel[x + dstOffset] = (refPel[x + refOffset] << shift) - IF_INTERNAL_OFFS;
          }

          for (int y = 0; y < sbHeight; y++, refPel += refStride, dstPel += dstStride)
          {
            dstPel[-1]      = (refPel[-1] << shift) - IF_INTERNAL_OFFS;
            dstPel[sbWidth] = (refPel[sbWidth] << shift) - IF_INTERNAL_OFFS;
          }

          // Configuração de parâmetros para o filtro de gradiente PROF
          const ptrdiff_t strideGrad = AFFINE_SUBBLOCK_WIDTH_EXT;

          g_pelBufOP.profGradFilter(dstExtBuf.buf, dstExtBuf.stride, AFFINE_SUBBLOCK_WIDTH_EXT,
                                    AFFINE_SUBBLOCK_HEIGHT_EXT, strideGrad, m_gradBuf[0], m_gradBuf[1], clpRng.bd);

          const Pel offset = (1 << shift >> 1) + IF_INTERNAL_OFFS;

          // Aplica o filtro de gradiente PROF no bloco predito
          Pel *src  = dst;
          Pel *gX   = m_gradBuf[0] + PROF_BORDER_EXT_H * strideGrad + PROF_BORDER_EXT_W;
          Pel *gY   = m_gradBuf[1] + PROF_BORDER_EXT_H * strideGrad + PROF_BORDER_EXT_W;
          Pel *dstY = dstBuf.bufAt(w, h);

          g_pelBufOP.applyPROF(dstY, dstBuf.stride, src, dstExtBuf.stride, sbWidth, sbHeight, gX, gY, strideGrad,
                               dMvScaleHor, dMvScaleVer, sbWidth, bi, shift, offset, clpRng);
        
        /*
        g_pelBufOP.applyPROF(dstY,         // Ponteiro para a área de destino (bloco predito)
                     dstBuf.stride, // Passo (stride) da área de destino
                     src,           // Ponteiro para a área de origem (bloco original)
                     dstExtBuf.stride, // Passo (stride) da área de origem extendida
                     sbWidth,       // Largura do bloco
                     sbHeight,      // Altura do bloco
                     gX,            // Gradiente na direção X
                     gY,            // Gradiente na direção Y
                     strideGrad,    // Passo (stride) do gradiente
                     dMvScaleHor,   // Escala do vetor de movimento na direção horizontal
                     dMvScaleVer,   // Escala do vetor de movimento na direção vertical
                     sbWidth,       // Largura do bloco (repetido, pode ser removido)
                     bi,            // Flag indicando se é uma predição bidirecional
                     shift,         // Deslocamento para normalização (baseado na profundidade de bits)
                     offset,        // Compensação para normalização (baseada na profundidade de bits)
                     clpRng);       // Faixa de valores válidos após aplicação do filtro (limiar de clipe)
        */
        
        }
      }
    }
  }
#if GDR_ENABLED
  return allOk;
#endif
}

void InterPrediction::applyBiOptFlow(const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths)
{
  // Inicialização de variáveis
  const int height = yuvDst.Y().height;
  const int width = yuvDst.Y().width;
  int heightG = height + 2 * BIO_EXTEND_SIZE;
  int widthG = width + 2 * BIO_EXTEND_SIZE;
  int offsetPos = widthG * BIO_EXTEND_SIZE + BIO_EXTEND_SIZE;

  // Inicialização de ponteiros para gradientes
  Pel* gradX0 = m_gradX0;
  Pel* gradX1 = m_gradX1;
  Pel* gradY0 = m_gradY0;
  Pel* gradY1 = m_gradY1;

  // Inicialização de variáveis de stride e buffers de origem e destino
  int stridePredMC = widthG + 2;
  const Pel* srcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + stridePredMC + 1;
  const Pel* srcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + stridePredMC + 1;
  const int src0Stride = stridePredMC;
  const int src1Stride = stridePredMC;

  Pel* dstY = yuvDst.Y().buf;
  const int dstStride = yuvDst.Y().stride;
  const Pel* srcY0Temp = srcY0;
  const Pel* srcY1Temp = srcY1;

  // Loop para processar cada lista de referência
  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    Pel* dstTempPtr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + stridePredMC + 1;
    Pel* gradY = (refList == 0) ? m_gradY0 : m_gradY1;
    Pel* gradX = (refList == 0) ? m_gradX0 : m_gradX1;

    // Aplica filtros de gradiente
    xBioGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY, clipBitDepths.recon[toChannelType(COMPONENT_Y)]);

    // Preenche os cantos da imagem filtrada temporária
    Pel* padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 2;
    for (int y = 0; y < height; y++)
    {
      padStr[-1] = padStr[0];
      padStr[width] = padStr[width - 1];
      padStr += stridePredMC;
    }

    padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 1;
    ::memcpy(padStr - stridePredMC, padStr, sizeof(Pel) * (widthG));
    ::memcpy(padStr + height * stridePredMC, padStr + (height - 1) * stridePredMC, sizeof(Pel) * (widthG));
  }

  // Configurações adicionais
  const ClpRng& clpRng = pu.cu->cs->slice->clpRng(COMPONENT_Y);
  const int bitDepth = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const int shiftNum = IF_INTERNAL_FRAC_BITS(bitDepth) + 1;
  const int offset = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
  const int limit = (1 << 4) - 1;

  int xUnit = (width >> 2);
  int yUnit = (height >> 2);

  // Inicialização de ponteiros
  Pel* dstY0 = dstY;
  gradX0 = m_gradX0; gradX1 = m_gradX1;
  gradY0 = m_gradY0; gradY1 = m_gradY1;

  // Loop para processar unidades de 4x4
  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
      // Inicialização de variáveis
      int tmpx = 0, tmpy = 0;
      int sumAbsGX = 0, sumAbsGY = 0, sumDIX = 0, sumDIY = 0;
      int sumSignGY_GX = 0;

      // Obtenção de ponteiros para gradientes e blocos de origem
      Pel* pGradX0Tmp = m_gradX0 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradX1Tmp = m_gradX1 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradY0Tmp = m_gradY0 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradY1Tmp = m_gradY1 + (xu << 2) + (yu << 2) * widthG;
      const Pel* SrcY1Tmp = srcY1 + (xu << 2) + (yu << 2) * src1Stride;
      const Pel* SrcY0Tmp = srcY0 + (xu << 2) + (yu << 2) * src0Stride;

      // Cálculo de somas de gradientes e diferenciais
      g_pelBufOP.calcBIOSums(SrcY0Tmp, SrcY1Tmp, pGradX0Tmp, pGradX1Tmp, pGradY0Tmp, pGradY1Tmp, xu, yu, src0Stride, src1Stride, widthG, bitDepth, &sumAbsGX, &sumAbsGY, &sumDIX, &sumDIY, &sumSignGY_GX);
      tmpx = (sumAbsGX == 0 ? 0 : rightShiftMSB(4 * sumDIX, sumAbsGX));
      tmpx = Clip3(-limit, limit, tmpx);

      const int tmpData = sumSignGY_GX * tmpx >> 1;

      tmpy = (sumAbsGY == 0 ? 0 : rightShiftMSB((4 * sumDIY - tmpData), sumAbsGY));
      tmpy = Clip3(-limit, limit, tmpy);

      // Atualização de ponteiros para processar blocos 4x4
      srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu * src0Stride + xu) << 2);
      srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu * src0Stride + xu) << 2);
      gradX0 = m_gradX0 + offsetPos + ((yu * widthG + xu) << 2);
      gradX1 = m_gradX1 + offsetPos + ((yu * widthG + xu) << 2);
      gradY0 = m_gradY0 + offsetPos + ((yu * widthG + xu) << 2);
      gradY1 = m_gradY1 + offsetPos + ((yu * widthG + xu) << 2);

      dstY0 = dstY + ((yu * dstStride + xu) << 2);

      // Aplicação do filtro BIO para média de 4x4
      xAddBIOAvg4(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY0, dstStride, gradX0, gradX1, gradY0, gradY1, widthG, (1 << 2), (1 << 2), (int)tmpx, (int)tmpy, shiftNum, offset, clpRng);
    }  // xu
  }  // yu
}
/*
  A função applyBiOptFlow realiza a aplicação do filtro de compensação de movimento bidirecional otimizado (BIO) para predição de blocos em um frame de vídeo.
  Inicializa variáveis e ponteiros necessários para o processamento.
  Aplica filtros de gradiente em cada lista de referência.
  Preenche os cantos da imagem filtrada temporária.
  Calcula as somas de gradientes e diferenciais para unidades de 4x4.
  Aplica o filtro BIO para a média de 4x4, considerando as informações calculadas anteriormente.
*/


void InterPrediction::xAddBIOAvg4(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  // Chama uma função externa para realizar a operação de média ponderada usando BIO para 4 pixels.
  g_pelBufOP.addBIOAvg4(src0, src0Stride, src1, src1Stride, dst, dstStride, gradX0, gradX1, gradY0, gradY1, gradStride, width, height, tmpx, tmpy, shift, offset, clpRng);
}

void InterPrediction::xBioGradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, int bitDepth)
{
  // Chama uma função externa para realizar o filtro BIO nos gradientes.
  g_pelBufOP.bioGradFilter(pSrc, srcStride, width, height, gradStride, gradX, gradY, bitDepth);
}

void InterPrediction::xCalcBIOPar(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, int bitDepth)
{
  // Chama uma função externa para calcular parâmetros BIO.
  g_pelBufOP.calcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, gradStride, widthG, heightG, bitDepth);
}

void InterPrediction::xCalcBlkGradient(int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize)
{
  // Chama uma função externa para calcular gradientes de blocos.
  g_pelBufOP.calcBlkGradient(sx, sy, arraysGx2, arraysGxGy, arraysGxdI, arraysGy2, arraysGydI, sGx2, sGy2, sGxGy, sGxdI, sGydI, width, height, unitSize);
}

// Função para realizar a média ponderada usando BIO para 4 pixels.
void InterPrediction::xWeightedAverage(const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0,
                                       const CPelUnitBuf &pcYuvSrc1, PelUnitBuf &pcYuvDst,
                                       const BitDepths &clipBitDepths, const ClpRngs &clpRngs, const bool bioApplied,
                                       bool lumaOnly, bool chromaOnly, PelUnitBuf *yuvDstTmp)
{
  // Verifica se os modos de croma e luma não estão ambos ativados.
  CHECK((chromaOnly && lumaOnly), "should not happen");

  // Obtém os índices de referência.
  const int refIdx0 = pu.refIdx[0];
  const int refIdx1 = pu.refIdx[1];

  // Caso ambos os índices de referência sejam válidos.
  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    // Verifica se BCW (Bi-Directional Compound with Weighting) está habilitado e se há buffer temporário disponível.
    if (pu.cu->bcwIdx != BCW_DEFAULT && (yuvDstTmp || !pu.ciipFlag))
    {
      // BCW não é permitido com BIO, portanto, verifica se BIO não foi aplicado.
      CHECK(bioApplied, "Bcw is disallowed with BIO");

      // Adiciona a média ponderada usando BCW.
      pcYuvDst.addWeightedAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, pu.cu->bcwIdx, chromaOnly, lumaOnly);

      // Se houver buffer temporário, adiciona a média normal no buffer temporário.
      if (yuvDstTmp)
        yuvDstTmp->addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly);

      return;
    }

    // Caso BIO seja aplicado.
    if (bioApplied)
    {
      // Configurações iniciais para BIO.
      const int src0Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const int src1Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const Pel *pSrcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + 2 * src0Stride + 2;
      const Pel *pSrcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + 2 * src1Stride + 2;

      // Verifica se BIO está habilitado.
      bool bioEnabled = true;

      // Se BIO estiver habilitado.
      if (bioEnabled)
      {
        // Aplica o fluxo otimizado de BIO.
        applyBiOptFlow(pu, pcYuvSrc0, pcYuvSrc1, refIdx0, refIdx1, pcYuvDst, clipBitDepths);

        // Se houver buffer temporário, adiciona a média normal no buffer temporário.
        if (yuvDstTmp)
          yuvDstTmp->bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);
      }
      // Se BIO estiver desabilitado.
      else
      {
        // Adiciona a média normal usando os blocos temporários.
        pcYuvDst.bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);

        // Se houver buffer temporário, copia o bloco luma para o buffer temporário.
        if (yuvDstTmp)
          yuvDstTmp->bufs[0].copyFrom(pcYuvDst.bufs[0]);
      }
    }

    // Se BIO não for aplicado e for apenas luma ou croma.
    if (!bioApplied && (lumaOnly || chromaOnly))
    {
      // Adiciona a média normal no bloco destino.
      pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly);
    }
    // Se BIO não for aplicado e não for apenas luma ou croma.
    else
    {
      // Adiciona a média normal no bloco destino considerando BIO.
      pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, bioApplied);
    }

    // Se houver buffer temporário.
    if (yuvDstTmp)
    {
      // Se BIO for aplicado.
      if (bioApplied)
      {
        // Se chroma estiver habilitado, copia os blocos chroma para o buffer temporário.
        if (isChromaEnabled(yuvDstTmp->chromaFormat))
        {
          yuvDstTmp->bufs[1].copyFrom(pcYuvDst.bufs[1]);
          yuvDstTmp->bufs[2].copyFrom(pcYuvDst.bufs[2]);
        }
      }
      // Se BIO não for aplicado.
      else
      {
        // Copia o bloco destino para o buffer temporário considerando luma e croma.
        yuvDstTmp->copyFrom(pcYuvDst, lumaOnly, chromaOnly);
      }
    }
  }
  // Caso apenas o primeiro índice de referência seja válido.
  else if (refIdx0 >= 0 && refIdx1 < 0)
  {
    // Se geoFlag estiver ativado, copia os blocos originais para o bloco destino.
    if (pu.cu->geoFlag)
    {
      pcYuvDst.copyFrom(pcYuvSrc0);
    }
    // Se geoFlag estiver desativado, copia os blocos originais para o bloco destino considerando clipping.
    else
    {
      pcYuvDst.copyClip(pcYuvSrc0, clpRngs, lumaOnly, chromaOnly);
    }

    // Se houver buffer temporário, copia os blocos para o buffer temporário considerando luma e croma.
    if (yuvDstTmp)
    {
      yuvDstTmp->copyFrom(pcYuvDst, lumaOnly, chromaOnly);
    }
  }
  // Caso apenas o segundo índice de referência seja válido.
  else if (refIdx0 < 0 && refIdx1 >= 0)
  {
    // Se geoFlag estiver ativado, copia os blocos originais para o bloco destino.
    if (pu.cu->geoFlag)
    {
      pcYuvDst.copyFrom(pcYuvSrc1);
    }
    // Se geoFlag estiver desativado, copia os blocos originais para o bloco destino considerando clipping.
    else
    {
      pcYuvDst.copyClip(pcYuvSrc1, clpRngs, lumaOnly, chromaOnly);
    }

    // Se houver buffer temporário, copia os blocos para o buffer temporário considerando luma e croma.
    if (yuvDstTmp)
    {
      yuvDstTmp->copyFrom(pcYuvDst, lumaOnly, chromaOnly);
    }
  }
}

// Função para realizar a compensação de movimento.
void InterPrediction::motionCompensation(PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList eRefPicList,
                                         const bool luma, const bool chroma, PelUnitBuf *predBufWOBIO)
{
  // Note: there appears to be an interaction with weighted prediction that
  // makes the code follow different paths if chroma is on or off (in the encoder).
  // Therefore for 4:0:0, "chroma" is not changed to false.
  // Verifica se o buffer temporário sem BIO foi fornecido em um caso que não deveria acontecer.
  CHECK(predBufWOBIO && pu.ciipFlag, "the case should not happen!");

  // Se não estiver no modo de codificador.
  if (!pu.cs->pcv->isEncoder)
  {
    // Se for IBC (Intra Block Copy), copia os blocos originais diretamente.
    if (CU::isIBC(*pu.cu))
    {
      CHECK(!luma, "IBC only for Chroma is not allowed.");
      xIntraBlockCopy(pu, predBuf, COMPONENT_Y);
      if (chroma && isChromaEnabled(pu.chromaFormat))
      {
        xIntraBlockCopy(pu, predBuf, COMPONENT_Cb);
        xIntraBlockCopy(pu, predBuf, COMPONENT_Cr);
      }
      return;
    }
  }
  // dual tree handling for IBC as the only ref
  // Tratamento de árvore dupla para IBC como a única referência.
  if ((!luma || !chroma) && eRefPicList == REF_PIC_LIST_0)
  {
    // Faz a predição unidirecional.
    xPredInterUni(pu, eRefPicList, predBuf, false, false, luma, chroma);
    return;
  }
  // else, go with regular MC below
  // Caso contrário, seguirá com MC regular abaixo
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  // Se não for IBC e (estiver usando WP para slices P ou WPBiPred para slices B).
  if( eRefPicList != REF_PIC_LIST_X )
  {
    // Se o buffer temporário sem BIO foi fornecido, realiza a predição ponderada.
    CHECK(predBufWOBIO != nullptr, "the case should not happen!");
    if (!CU::isIBC(*pu.cu) && ((sliceType == P_SLICE && pps.getUseWP()) || (sliceType == B_SLICE && pps.getWPBiPred())))
    {
      // Faz a predição unidirecional.
      xPredInterUni(pu, eRefPicList, predBuf, true, false, luma, chroma);
      
      //Faz a predição ponderada.
      xWeightedPredictionUni(pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred, (luma && !chroma),
                             (!luma && chroma));
    }
    else
    {
      // Faz a predição unidirecional padrão.
      xPredInterUni(pu, eRefPicList, predBuf, false, false, luma, chroma);
    }
  }
  // Caso contrário, trata-se de um bloco de referência dupla.
  else
  {
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];
    
    // Verificações adicionais para blocos bipreditos de 4x8/8x4.
    CHECK(!pu.cu->affine && refIdx0 >= 0 && refIdx1 >= 0 && pu.lwidth() + pu.lheight() == 12,
          "invalid 4x8/8x4 bi-predicted blocks");

    // Verifica se a aplicação de BIO (Bi-Directional Optical Flow) é possível.
    bool bioApplied = false;
    if (pu.cs->sps->getBDOFEnabledFlag() && !pu.cs->picHeader->getBdofDisabledFlag())
    {
      if (pu.cu->affine || m_subPuMC)
      {
        bioApplied = false;
      }
      else
      {
        bioApplied = PU::isSimpleSymmetricBiPred(pu) && PU::dmvrBdofSizeCheck(pu) && !pu.ciipFlag && !pu.cu->smvdMode;
      }

      if (pu.mmvdEncOptMode == 2 && pu.mmvdMergeFlag)
      {
        bioApplied = false;
      }
    }

    // Verifica se as referências estão escaladas.
    bool refIsScaled = ( refIdx0 < 0 ? false : pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps ) ) ||
                       ( refIdx1 < 0 ? false : pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps ) );
    bioApplied = refIsScaled ? false : bioApplied;
    // Verifica se DMVR (Differential Motion Vector Refinement) é aplicável.
    bool dmvrApplied = (pu.mvRefine) && PU::checkDMVRCondition(pu);
    
    // Se a aplicação de BIO não for possível e DMVR não for aplicado.
    if ((pu.lumaSize().width > MAX_BDOF_APPLICATION_REGION || pu.lumaSize().height > MAX_BDOF_APPLICATION_REGION) && pu.mergeType != MRG_TYPE_SUBPU_ATMVP && (bioApplied && !dmvrApplied))
    {
      // Realiza a sub-predição usando BIO.
      xSubPuBio(pu, predBuf, eRefPicList, predBufWOBIO);
    }
    else
    {
      // Se o tipo de merge for diferente do padrão ou IBC.
      if (pu.mergeType != MRG_TYPE_DEFAULT_N && pu.mergeType != MRG_TYPE_IBC)
      {
        // Se o buffer temporário sem BIO foi fornecido, realiza a sub-predição usando bloco de referência único.
        CHECK(predBufWOBIO != nullptr, "the case should not happen!");
        xSubPuMC(pu, predBuf, eRefPicList, luma, chroma);
      }
      // Se o movimento for idêntico.
      else if (xCheckIdenticalMotion(pu))
      {
        //Faz a predição unidirecional
        xPredInterUni(pu, REF_PIC_LIST_0, predBuf, false, false, luma, chroma);
        
        // Se o buffer temporário sem BIO foi fornecido, copia os blocos para o buffer sem alterações.
        if (predBufWOBIO)
        {
          predBufWOBIO->copyFrom(predBuf, (luma && !chroma), (chroma && !luma));
        }
      }
      // Caso contrário, realiza a predição bipredita.
      else
      {
        xPredInterBi(pu, predBuf, luma, chroma, predBufWOBIO);
      }
    }
  }
  return;
}

// Função para realizar a compensação de movimento para uma Coding Unit (CU) específica.
void InterPrediction::motionCompensateCu(CodingUnit &cu, const RefPicList eRefPicList, const bool luma,
                                         const bool chroma)
{
  // Itera sobre todas as Partições de Unidade (PUs) dentro da CU.
  for( auto &pu : CU::traversePUs( cu ) )
  {
    // Obtém o buffer de predição para a PU.
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
    
    // Habilita o refinamento de vetor de movimento para a PU.
    pu.mvRefine = true;
    
    // Chama a função de compensação de movimento para a PU.
    motionCompensation(pu, predBuf, eRefPicList, luma, chroma, nullptr);
    
    // Desabilita o refinamento de vetor de movimento para a PU.
    pu.mvRefine = false;
  }
}

// Função responsável por realizar a compensação de movimento para uma Prediction Unit específica. Ela é uma interface que simplifica a chamada para a função principal de compensação de movimento.
void InterPrediction::motionCompensatePu(PredictionUnit &pu, const RefPicList eRefPicList, const bool luma,
                                         const bool chroma)
{
  // Obtém o buffer de predição para a Prediction Unit (pu)
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  // Chama a função de compensação de movimento com os parâmetros apropriados
  motionCompensation(pu, predBuf, eRefPicList, luma, chroma, nullptr);
}

// Função para realizar um deslocamento para a direita usando o bit mais significativo do numerador em relação ao denominador.
int InterPrediction::rightShiftMSB(int numer, int denom)
{
  // Realiza o deslocamento para a direita pelo logaritmo de base 2 do denominador.
  return numer >> floorLog2(denom);
}

// Função para realizar a compensação de movimento para blocos geométricos.
void InterPrediction::motionCompensationGeo(CodingUnit &cu, MergeCtx &geoMrgCtx)
{
  // Obtém as informações sobre a divisão geométrica, índices de candidatos e outras informações importantes.
  const uint8_t splitDir = cu.firstPU->geoSplitDir;
  const uint8_t candIdx0 = cu.firstPU->geoMergeIdx0;
  const uint8_t candIdx1 = cu.firstPU->geoMergeIdx1;

  // Itera sobre as PUs dentro da CU.
  for (auto &pu : CU::traversePUs(cu))
  {
    // Cria buffers temporários para as partes geométricas.
    const UnitArea localUnitArea(cu.cs->area.chromaFormat, Area(0, 0, pu.lwidth(), pu.lheight()));
    PelUnitBuf tmpGeoBuf0 = m_geoPartBuf[0].getBuf(localUnitArea);
    PelUnitBuf tmpGeoBuf1 = m_geoPartBuf[1].getBuf(localUnitArea);
    PelUnitBuf predBuf = cu.cs->getPredBuf(pu);

    // Configura as informações de merge para a PU e expande as informações de movimento.
    geoMrgCtx.setMergeInfo(pu, candIdx0);
    PU::spanMotionInfo(pu);
    
    // Realiza a compensação de movimento para a primeira parte geométrica.
    // TODO: check 4:0:0 interaction with weighted prediction.
    motionCompensation(pu, tmpGeoBuf1, REF_PIC_LIST_X, true, isChromaEnabled(pu.chromaFormat), nullptr);
    
    // Verifica se há uma violação da restrição MCTS no modo decodificador.
    if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint( pu, true ) )
    {
      printf( "DECODER_GEO_PU: pu motion vector across tile boundaries (%d,%d,%d,%d)\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight() );
    }

    // Realiza a combinação ponderada dos blocos geométricos.
    weightedGeoBlk(pu, splitDir, isChromaEnabled(pu.chromaFormat)? MAX_NUM_CHANNEL_TYPE : CHANNEL_TYPE_LUMA, predBuf, tmpGeoBuf0, tmpGeoBuf1);
  }
}

// Função para realizar a combinação ponderada de blocos geométricos.
void InterPrediction::weightedGeoBlk( PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  // Verifica o tipo de canal e chama a função correspondente na interface de interpolação.
  if( channel == CHANNEL_TYPE_LUMA )
  {
    m_if.weightedGeoBlk( pu, pu.lumaSize().width, pu.lumaSize().height, COMPONENT_Y, splitDir, predDst, predSrc0, predSrc1 );
  }
  else if( channel == CHANNEL_TYPE_CHROMA )
  {
    m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0, predSrc1 );
    m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0, predSrc1 );
  }
  else
  {
    m_if.weightedGeoBlk( pu, pu.lumaSize().width,   pu.lumaSize().height,   COMPONENT_Y,  splitDir, predDst, predSrc0, predSrc1 );
    // Verifica se a croma está habilitada antes de chamar as funções para os canais croma.
    if (isChromaEnabled(pu.chromaFormat))
    {
      m_if.weightedGeoBlk(pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0,
                          predSrc1);
      m_if.weightedGeoBlk(pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0,
                          predSrc1);
    }
  }
}

// Função para pré-buscar blocos de pixels para a predição utilizando DMVR.
void InterPrediction::xPrefetch(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma)
{
  int offset, width, height;
  Mv cMv;

  // Obtém o ponteiro para a imagem de referência desejada.
  const Picture* refPic = pu.cu->slice->getRefPic( refId, pu.refIdx[refId] )->unscaledPic;
  int mvShift = (MV_FRACTIONAL_BITS_INTERNAL);

  // Determina a faixa de componentes a serem processadas com base no parâmetro "forLuma".
  int start = 0;
  int end = MAX_NUM_COMPONENT;

  start = forLuma ? 0 : 1;
  end = forLuma ? 1 : MAX_NUM_COMPONENT;

  // Loop através dos componentes para pré-buscar os pixels necessários.
  for (int compID = start; compID < end; compID++)
  {
    // Obtém o vetor de movimento correspondente ao componente atual.
    cMv = Mv(pu.mv[refId].getHor(), pu.mv[refId].getVer());
    
    // Configurações específicas do componente, como tamanho do filtro e ajustes de escala.
    pcPad.bufs[compID].stride = (pcPad.bufs[compID].width + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA);
    int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
    width = pcPad.bufs[compID].width;
    height = pcPad.bufs[compID].height;
    offset = (DMVR_NUM_ITERATION) * (pcPad.bufs[compID].stride + 1);
    int mvshiftTempHor = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
    int mvshiftTempVer = mvShift + getComponentScaleY((ComponentID)compID, pu.chromaFormat);
    width += (filtersize - 1);
    height += (filtersize - 1);
    
    // Ajusta o vetor de movimento com base no tamanho do filtro.
    cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTempHor),
      -(((filtersize >> 1) - 1) << mvshiftTempVer));
    
    // Verifica se a referência deve ser embrulhada (wrap-around) ou apenas cortada (clip).
    bool wrapRef = false;
    
    /* Se a referência estiver configurada para suportar "wrap-around", a função wrapClipMv 
    será chamada para aplicar a estratégia de "wrap-around". Caso contrário, a função clipMv 
    será chamada para aplicar a estratégia de "clipping".
    */
    if( refPic->isWrapAroundEnabled( pu.cs->pps ) )
    {
      wrapRef = wrapClipMv( cMv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps );
    }
    else
    {
      clipMv( cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps );
    }
    /* Pre-fetch similar to HEVC*/
    {
      // Pré-busca os pixels necessários para a predição.
      CPelBuf refBuf;
      Position Rec_offset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTempHor, cMv.getVer() >> mvshiftTempVer);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, Rec_offset, pu.blocks[compID].size()), wrapRef);
      PelBuf &dstBuf = pcPad.bufs[compID];
      g_pelBufOP.copyBuffer((Pel *)refBuf.buf, refBuf.stride, ((Pel *)dstBuf.buf) + offset, dstBuf.stride, width, height);
    }
  }
}

// Função para realizar o preenchimento (padding) dos pixels para predição utilizando DMVR.
void InterPrediction::xPad(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId)
{
  int offset = 0, width, height;
  int padsize;
  Mv cMv;
  
  // Loop através dos componentes válidos para o formato de croma da PredictionUnit.
  for (int compID = 0; compID < getNumberValidComponents(pu.chromaFormat); compID++)
  {
    // Configurações específicas do componente, como tamanho do filtro e ajustes de escala.
    int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
    width = pcPad.bufs[compID].width;
    height = pcPad.bufs[compID].height;
    offset = (DMVR_NUM_ITERATION) * (pcPad.bufs[compID].stride + 1);

    // Obtém o tamanho do preenchimento considerando a escala do componente.
    /*using the larger padsize for 422*/
    padsize = (DMVR_NUM_ITERATION) >> getComponentScaleY((ComponentID)compID, pu.chromaFormat);
    width += (filtersize - 1);
    height += (filtersize - 1);

    // Realiza o preenchimento em todos os lados com o tamanho especificado.
    /*padding on all side of size DMVR_PAD_LENGTH*/
    g_pelBufOP.padding(pcPad.bufs[compID].buf + offset, pcPad.bufs[compID].stride, width, height, padsize);
  }
}

// Função para realizar uma divisão aproximada otimizada por potência de 2 (shift).
inline int32_t div_for_maxq7(int64_t N, int64_t D)
{
  int32_t sign, q;

  // Verifica o sinal e ajusta os operandos, se necessário.
  sign = 0;
  if (N < 0)
  {
    sign = 1;
    N = -N;
  }

  q = 0;

  // Fase 1 da divisão (shift de 3 bits para a esquerda).
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  // Fase 2 da divisão (shift de 1 bit para a direita).
  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  // Fase 3 da divisão (shift de 1 bit para a direita).
  if (N >= (D >> 1))
  {
    q++;
  }

  // Retorna o resultado considerando o sinal.
  return sign ? -q : q;
}

void xSubPelErrorSrfc(uint64_t *sadBuffer, int32_t *deltaMv)
{
  // Variáveis para armazenar o numerador e denominador dos cálculos
  int64_t numerator, denominator;
  
  // Variável para armazenar o deslocamento do subpíxel do vetor de movimento
  int32_t mvDeltaSubPel;
  
  // Nível de subpíxel do vetor de movimento (1: half pel, 2: Qpel, 3:1/8, 4: 1/16)
  int32_t mvSubPelLvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
  
                                                        /*horizontal*/
  // Cálculos para a direção horizontal                                                      
  numerator   = (int64_t)((sadBuffer[1] - sadBuffer[3]) << mvSubPelLvl);
  denominator = (int64_t)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

  // Verificação se o denominador é diferente de zero
  if (0 != denominator)
  {
    // Verificação adicional para evitar divisão por zero
    if ((sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]))
    {
      // Cálculo do deslocamento do subpíxel usando uma função externa div_for_maxq7
      mvDeltaSubPel = div_for_maxq7(numerator, denominator);
      // Armazenamento do resultado na primeira componente do vetor deltaMv
      deltaMv[0]    = (mvDeltaSubPel);
    }
    else
    {
      // Caso especial: se sadBuffer[1] for igual a sadBuffer[0]
      if (sadBuffer[1] == sadBuffer[0])
      {
        // Atribui -8 se sadBuffer[1] for diferente de sadBuffer[0], senão, atribui 8
        deltaMv[0] = -8;   // half pel
      }
      else
      {
        deltaMv[0] = 8;   // half pel
      }
    }
  }

  /*vertical*/   // Cálculos para a direção vertical
  numerator   = (int64_t)((sadBuffer[2] - sadBuffer[4]) << mvSubPelLvl);
  denominator = (int64_t)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));

  // Verificação se o denominador é diferente de zero
  if (0 != denominator)
  {
    // Verificação adicional para evitar divisão por zero
    if ((sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]))
    {
      // Cálculo do deslocamento do subpíxel usando a função externa div_for_maxq7
      mvDeltaSubPel = div_for_maxq7(numerator, denominator);
      // Armazenamento do resultado na segunda componente do vetor deltaMv
      deltaMv[1]    = (mvDeltaSubPel);
    }
    else
    {
      // Caso especial: se sadBuffer[2] for igual a sadBuffer[0]
      // Atribui -8 se sadBuffer[2] for diferente de sadBuffer[0], senão, atribui 8
      if (sadBuffer[2] == sadBuffer[0])
      {
        deltaMv[1] = -8;   // half pel
      }
      else
      {
        deltaMv[1] = 8;   // half pel
      }
    }
  }
  return;
}

/*
Essa função é responsável por refinar o vetor de movimento bidirecional (BIPMV) usando uma busca em uma grade de offsets. Ela itera sobre 25 offsets diferentes,
calcula o custo para cada offset usando a função xDMVRCost e atualiza o vetor deltaMV com o offset que resultou no menor custo (minCost). A função também armazena
os custos calculados em um array (pSADsArray) para evitar recálculos redundantes.
*/
void InterPrediction::xBIPMVRefine(int bd, Pel *pRefL0, Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray, int width, int height)
{
  // Obtém as informações sobre o stride dos blocos de referência
  const int32_t refStrideL0 = m_biLinearBufStride;
  const int32_t refStrideL1 = m_biLinearBufStride;

  // Armazena os ponteiros originais para as referências L0 e L1
  Pel *pRefL0Orig = pRefL0;
  Pel *pRefL1Orig = pRefL1;

  // Loop para iterar sobre 25 offsets diferentes
  for (int nIdx = 0; (nIdx < 25); ++nIdx)
  {
    // Calcula o offset para o índice atual
    int32_t sadOffset = ((m_pSearchOffset[nIdx].getVer() * ((2 * DMVR_NUM_ITERATION) + 1)) + m_pSearchOffset[nIdx].getHor());
    
    // Atualiza os ponteiros para as referências L0 e L1 de acordo com o offset
    pRefL0 = pRefL0Orig + m_pSearchOffset[nIdx].hor + (m_pSearchOffset[nIdx].ver * refStrideL0);
    pRefL1 = pRefL1Orig - m_pSearchOffset[nIdx].hor - (m_pSearchOffset[nIdx].ver * refStrideL1);
    
    // Verifica se o custo para este offset já foi calculado anteriormente
    if (*(pSADsArray + sadOffset) == MAX_UINT64)
    {
      // Se ainda não foi calculado, calcula o custo usando a função xDMVRCost
      const uint64_t cost = xDMVRCost(bd, pRefL0, refStrideL0, pRefL1, refStrideL1, width, height);
      *(pSADsArray + sadOffset) = cost;
    }

    // Verifica se o custo atual é menor que o custo mínimo
    if (*(pSADsArray + sadOffset) < minCost)
    {
      // Se for menor, atualiza o custo mínimo e os deltas de movimento
      minCost = *(pSADsArray + sadOffset);
      deltaMV[0] = m_pSearchOffset[nIdx].getHor();
      deltaMV[1] = m_pSearchOffset[nIdx].getVer();
    }
  }
}

/*
Esta função realiza a predição final de blocos para a técnica de Refinamento Dinâmico de Vetor de Movimento (DMVR).
A função itera sobre as listas de referência (L0 e L1), calcula a MV inicial para a fusão e itera sobre os componentes 
de cor (Y, Cb, Cr) para realizar a previsão do bloco usando a função xPredInterBlk. Além disso, a função lida com o 
preenchimento do bloco em caso de movimento e verificações específicas relacionadas à MCTS (Simulação de Testes de 
Conformidade MCTS).
A função também faz uso de diversas constantes e funções específicas do codificador VVC.
*/
void InterPrediction::xFinalPaddedMCForDMVR(PredictionUnit &pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1,
                                            PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied,
                                            const Mv mergeMV[NUM_REF_PIC_LIST_01], bool blockMoved)
{
  int offset, deltaIntMvX, deltaIntMvY;

  // Inicializa buffers temporários para YUV e preenchimento
  PelUnitBuf pcYUVTemp = pcYuvSrc0;
  PelUnitBuf pcPadTemp = pcPad0;
  /*always high precision MVs are used*/
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  // Itera sobre as listas de referência (L0 e L1)
  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    Mv cMv = pu.mv[refId];
    m_iRefListIdx = refId;

    // Obtém a referência correspondente
    const Picture* refPic = pu.cu->slice->getRefPic( refId, pu.refIdx[refId] )->unscaledPic;
    
    // Obtém MVs inicializados para a fusão
    Mv cMvClipped = cMv;
    if( !pu.cs->pps->getWrapAroundEnabledFlag() )
    {
      clipMv( cMvClipped, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps );
    }

    Mv startMv = mergeMV[refId];

    // Verifica a restrição de MCTS para MV
    if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvForMCTSConstraint( pu, startMv, MV_PRECISION_INTERNAL ) )
    {
      const Area& tileArea = pu.cs->picture->mctsInfo.getTileArea();
      printf( "Attempt an access over tile boundary at block %d,%d %d,%d with MV %d,%d (in Tile TL: %d,%d BR: %d,%d)\n",
        pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), startMv.getHor(), startMv.getVer(), tileArea.topLeft().x, tileArea.topLeft().y, tileArea.bottomRight().x, tileArea.bottomRight().y );
      THROW( "MCTS constraint failed!" );
    }

    // Itera sobre os componentes de cor (Y, Cb, Cr)
    for (int compID = 0; compID < getNumberValidComponents(pu.chromaFormat); compID++)
    {
      Pel *srcBufPelPtr = nullptr;
      int pcPadstride = 0;

      // Verifica se o bloco se moveu ou se é o componente Y
      if (blockMoved || (compID == 0))
      {
        pcPadstride = pcPadTemp.bufs[compID].stride;
        int mvshiftTempHor = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
        int mvshiftTempVer = mvShift + getComponentScaleY((ComponentID)compID, pu.chromaFormat);
        int leftPixelExtra;

        // Define a quantidade de pixels extras à esquerda com base no componente
        if (compID == COMPONENT_Y)
        {
          leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
        }
        else
        {
          leftPixelExtra = (NTAPS_CHROMA >> 1) - 1;
        }
        PelBuf &srcBuf = pcPadTemp.bufs[compID];
        deltaIntMvX    = (cMv.getHor() >> mvshiftTempHor) - (startMv.getHor() >> mvshiftTempHor);
        deltaIntMvY    = (cMv.getVer() >> mvshiftTempVer) - (startMv.getVer() >> mvshiftTempVer);

        CHECK((abs(deltaIntMvX) > DMVR_NUM_ITERATION) || (abs(deltaIntMvY) > DMVR_NUM_ITERATION), "not expected DMVR movement");

        // Calcula o offset no buffer de preenchimento
        offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (pcPadTemp.bufs[compID].stride + 1);
        offset += (deltaIntMvY)* pcPadTemp.bufs[compID].stride;
        offset += (deltaIntMvX);
        srcBufPelPtr = (srcBuf.buf + offset);
      }

      // Desativa temporariamente a previsão de cache para o componente atual
      JVET_J0090_SET_CACHE_ENABLE(false);
      
      // Realiza a previsão do bloco usando xPredInterBlk
      xPredInterBlk((ComponentID) compID, pu, refPic, cMvClipped, pcYUVTemp, true,
                    pu.cs->slice->getClpRngs().comp[compID], bioApplied, false,
                    pu.cu->slice->getScalingRatio(refId, pu.refIdx[refId]), 0, 0, 0, srcBufPelPtr, pcPadstride);
      
      // Restaura a previsão de cache
      JVET_J0090_SET_CACHE_ENABLE(false);
    }

    // Atualiza os buffers temporários para a próxima iteração
    pcYUVTemp = pcYuvSrc1;
    pcPadTemp = pcPad1;
  }
}

/*
A função xDMVRCost calcula o custo (distância) entre dois blocos de pixels para a técnica de 
Refinamento Dinâmico de Vetor de Movimento (DMVR).
A função utiliza a estrutura 'DistParam' para configurar parâmetros necessários para o cálculo
da distância entre os blocos de pixels. Parâmetros incluem informações sobre blocos de origem e 
referência, profundidade de bits (bit depth), largura e altura dos blocos etc.
*/
uint64_t InterPrediction::xDMVRCost(int bitDepth, Pel* pOrg, uint32_t refStride, const Pel* pRef, uint32_t orgStride, int width, int height)
{
  // Cria uma estrutura DistParam para parâmetros de cálculo de distância
  DistParam cDistParam;

  //Configuração de parâmetros
  cDistParam.applyWeight = false; //Não aplicar pesos
  cDistParam.useMR = false; //Não usar remoção de movimento
  
  //objeto de custo de codificação/distância
  m_pcRdCost->setDistParam(cDistParam, pOrg, pRef, orgStride, refStride, bitDepth, COMPONENT_Y, width, height, 1);
  
  // distFunc: Calcula o custo entre os blocos usando a função de distância configurada
  uint64_t uiCost = cDistParam.distFunc(cDistParam);

/*
 Divide o custo por 2 (shift right 1) e retorna um valor uint64_t (valor 
 frequentemente usado em algoritmos de otimização para tomada de decisões sobre 
 a melhor escolha de vetores de movimento)
*/
  return uiCost>>1;
}

/*
A função xDMVRSubPixelErrorSurface calcula a contribuição do refinamento dinâmico de 
vetores de movimento (DMVR) para o erro subpíxel da superfície, com base nos valores 
de SAD (Soma das Diferenças Absolutas) armazenados em um array pSADsArray.
Essa função realiza o refinamento subpíxel com base em informações de custo e armazena 
os resultados nos arrays totalDeltaMV e deltaMV. O refinamento subpíxel é condicional, 
ocorrendo apenas se o custo não for zero e os deslocamentos deltaMV não forem iguais a 
2 em MV_FRACTIONAL_BITS_INTERNAL.
*/
void xDMVRSubPixelErrorSurface(bool notZeroCost, int16_t *totalDeltaMV, int16_t *deltaMV, uint64_t *pSADsArray)
{
  //Tamanho da linha da matriz de SADs
  int sadStride = (((2 * DMVR_NUM_ITERATION) + 1));
  
  //Buffer para armazenar SADs
  uint64_t sadbuffer[5];

  // Verifica se o custo não é zero e se os deslocamentos deltaMV não são iguais a 2 em MV_FRACTIONAL_BITS_INTERNAL
  if (notZeroCost && (abs(totalDeltaMV[0]) != (2 << MV_FRACTIONAL_BITS_INTERNAL))
    && (abs(totalDeltaMV[1]) != (2 << MV_FRACTIONAL_BITS_INTERNAL)))
  {
    // Vetor temporário para armazenar deltaMVs temporários
    int32_t tempDeltaMv[2] = { 0,0 };

    // Preenche o buffer com os valores de SAD apropriados
    sadbuffer[0] = pSADsArray[0];
    sadbuffer[1] = pSADsArray[-1];
    sadbuffer[2] = pSADsArray[-sadStride];
    sadbuffer[3] = pSADsArray[1];
    sadbuffer[4] = pSADsArray[sadStride];

    // Chama a função de subpíxel para calcular os deltaMVs apropriados
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);

    // Atualiza os deltaMVs totais
    totalDeltaMV[0] += tempDeltaMv[0];
    totalDeltaMV[1] += tempDeltaMv[1];
  }
}

/// @brief A função xinitMC realiza a inicialização da predição de movimento(?)/compensação de movimento 
///(MC) para a técnica de refinamento de vetores de movimento dinâmico (DMVR).
///Essa função prepara as imagens de referência para a predição de movimento em blocos DMVR. 
///Os vetores de movimento iniciais são ajustados e a predição de movimento é realizada para 
///as referências L0 e L1, resultando em duas áreas de predição temporárias yuvPredTempL0 e 
///yuvPredTempL1. Essas áreas são usadas posteriormente durante o processo de refinamento DMVR.
/// @param pu 
/// @param clpRngs 
void InterPrediction::xinitMC(PredictionUnit& pu, const ClpRngs &clpRngs)
{
  // Obtém os índices de referência
  const int refIdx0 = pu.refIdx[0];
  const int refIdx1 = pu.refIdx[1];

  /*use merge MV as starting MV*/
  // Utiliza os vetores de movimento de merge como vetores iniciais
  Mv mergeMVL0(pu.mv[REF_PIC_LIST_0]);
  Mv mergeMVL1(pu.mv[REF_PIC_LIST_1]);

  /*Clip the starting MVs*/
  // Limita os vetores de movimento iniciais se a wrap-around não estiver ativado
  if( !pu.cs->pps->getWrapAroundEnabledFlag() )
  {
    clipMv( mergeMVL0, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps );
    clipMv( mergeMVL1, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps );
  }

  /*L0 MC for refinement*/
  // Inicializa a predição de movimento para L0 (refinamento)
  {
    int offset;
    int leftPixelExtra = (NTAPS_LUMA >> 1) - 1;

    // Calcula o deslocamento para o canto superior esquerdo do bloco
    offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y].stride + 1);
    offset += (-(int)DMVR_NUM_ITERATION)* (int)m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y].stride;
    offset += (-(int)DMVR_NUM_ITERATION);

    // Obtendo um ponteiro para a área de origem no buffer de referência L0
    PelBuf srcBuf = m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y];

    // Configura a área de destino para o bloco de predição temporária
    PelUnitBuf yuvPredTempL0 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL0,
      m_biLinearBufStride
      , pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION)));

    // Realiza a predição de movimento para L0
    // Realizando a predição do bloco com Motion Compensation (MC) usando a referência L0
    xPredInterBlk( COMPONENT_Y, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->unscaledPic, mergeMVL0, yuvPredTempL0, true, clpRngs.comp[COMPONENT_Y],
      false, false, pu.cu->slice->getScalingRatio( REF_PIC_LIST_0, refIdx0 ), pu.lwidth() + ( 2 * DMVR_NUM_ITERATION ), pu.lheight() + ( 2 * DMVR_NUM_ITERATION ), true, ( (Pel *)srcBuf.buf ) + offset, srcBuf.stride );
  }

  /*L1 MC for refinement*/
  // Inicializa a predição de movimento para L1 (refinamento)
  {
    int offset;
    int leftPixelExtra = (NTAPS_LUMA >> 1) - 1;

    // Calcula o deslocamento para o canto superior esquerdo do bloco
    offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y].stride + 1);
    offset += (-(int)DMVR_NUM_ITERATION)* (int)m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y].stride;
    offset += (-(int)DMVR_NUM_ITERATION);

    // Obtendo um ponteiro para a área de origem no buffer de referência L1
    PelBuf srcBuf = m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y];
   
    // Configura a área de destino para o bloco de predição temporária
    PelUnitBuf yuvPredTempL1 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL1,
      m_biLinearBufStride
      , pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION)));

    // Realiza a predição de movimento para L1
    // Realizando a predição do bloco com Motion Compensation (MC) usando a referência L1
    xPredInterBlk( COMPONENT_Y, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->unscaledPic, mergeMVL1, yuvPredTempL1, true, clpRngs.comp[COMPONENT_Y],
      false, false, pu.cu->slice->getScalingRatio( REF_PIC_LIST_1, refIdx1 ), pu.lwidth() + ( 2 * DMVR_NUM_ITERATION ), pu.lheight() + ( 2 * DMVR_NUM_ITERATION ), true, ( (Pel *)srcBuf.buf ) + offset, srcBuf.stride );
  }
}

/// @brief 
/// @param pu 
/// @param pcYuvDst 
/// @param clpRngs 
/// @param bioApplied 
void InterPrediction::xProcessDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied)
{
  // Número de iterações do processo DMVR
  int iterationCount = 1;

  /*Always High Precision*/
  // Sempre utiliza alta precisão para o MV
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  /*use merge MV as starting MV*/
  // Usa MV do merge como MV inicial
  Mv mergeMv[] = { pu.mv[REF_PIC_LIST_0] , pu.mv[REF_PIC_LIST_1] };
  
  // Largura do buffer biLinear
  m_biLinearBufStride = (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION));

  // Tamanho do bloco DMVR em altura e largura
  int dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
  int dx = std::min<int>(pu.lumaSize().width,  DMVR_SUBCU_WIDTH);
  
  // Posição do bloco DMVR
  Position puPos = pu.lumaPos();
  
  // Precisão do bitdepth
  int bd = pu.cs->slice->getClpRngs().comp[COMPONENT_Y].bd;

  // Limite para ativar o modo BIO (Bi-Orthogonal)
  int            bioEnabledThres = 2 * dy * dx;

  // Array para armazenar o tipo de aplicação BIO para cada subbloco DMVR
  bool           bioAppliedType[MAX_NUM_SUBCU_DMVR];

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  // Ativa o modo de medição de largura de banda de memória
  JVET_J0090_SET_CACHE_ENABLE(true);

  // Loop sobre as listas de referência (0 e 1)
  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);

    // Loop sobre as componentes de cor (Y, Cb, Cr)
    for (int compID = 0; compID < MAX_NUM_COMPONENT; compID++)
    {
      // Obtém o vetor de movimento (MV) da unidade de predição
      Mv cMv = pu.mv[refId];

      // Calcula o deslocamento para MV e o tamanho do filtro
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
      int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
      
      // Atualiza o vetor de movimento para levar em conta o deslocamento do filtro
      cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTemp), -(((filtersize >> 1) - 1) << mvshiftTemp));
      
      // Verifica se o modo de referência circular (wrap) está ativado
      bool wrapRef = false;
      if ( pu.cs->pps->getWrapAroundEnabledFlag() )
      {
        wrapRef = wrapClipMv(cMv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps);
      }
      else
      {
        clipMv(cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
      }

      // Calcula a largura e altura da região de referência
      int width = pcYuvDst.bufs[compID].width + (filtersize - 1);
      int height = pcYuvDst.bufs[compID].height + (filtersize - 1);

      // Obtém o buffer de referência
      CPelBuf refBuf;
      Position recOffset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTemp, cMv.getVer() >> mvshiftTemp);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, recOffset, pu.blocks[compID].size()), wrapRef);

      // Configura a referência para fins de medição
      JVET_J0090_SET_REF_PICTURE(refPic, (ComponentID)compID);
      // Loop para acessar cada elemento do buffer de referência
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          // Medição de acesso ao cache para este elemento
          JVET_J0090_CACHE_ACCESS(((Pel *)refBuf.buf) + row * refBuf.stride + col, __FILE__, __LINE__);
        }
      }
    }
  }
  // Desativa o modo de medição de largura de banda de memória
  JVET_J0090_SET_CACHE_ENABLE(false);
#endif

  {
    /*
    Nesse código, buffers de predição e referência são configurados para serem utilizados na compensação de 
    movimento após a aplicação do DMVR (compensação de movimento com vetor de movimento dinâmico). Os buffers 
    são inicializados com base nas propriedades da unidade de predição (pu) e na configuração de DMVR. As 
    operações incluem cálculos de escalas, configuração de buffers e criação de subunidades para regiões 
    específicas de predição.
    */
    
    // Inicialização da variável 'num' com valor zero (OK Braulio ¬¬)
    int num = 0;

    // Obtém os fatores de escala para as componentes de croma (Cb e Cr?)
    int scaleX = getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
    int scaleY = getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);

    // Calcula a largura do buffer usando o número de iterações DMVR
    m_biLinearBufStride = (dx + (2 * DMVR_NUM_ITERATION));
    
    // point mc buffer to cetre point to avoid multiplication to reach each iteration to the begining
    // Aponta os buffers de predição linear para o centro, evitando multiplicação para acessar cada iteração no início
    Pel *biLinearPredL0 = m_cYuvPredTempDMVRL0 + (DMVR_NUM_ITERATION * m_biLinearBufStride) + DMVR_NUM_ITERATION;
    Pel *biLinearPredL1 = m_cYuvPredTempDMVRL1 + (DMVR_NUM_ITERATION * m_biLinearBufStride) + DMVR_NUM_ITERATION;

    // Cria uma subunidade de predição (subPu) baseada na unidade de predição (pu)
    PredictionUnit subPu = pu;
    subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(puPos.x, puPos.y, dx, dy)));
    
    // Inicializa os buffers de referência para a Lista 0 (L0) após a compensação de movimento DMVR
    m_cYuvRefBuffDMVRL0 = (pu.chromaFormat == CHROMA_400 ?
      PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL0[0], pcYuvDst.Y())) :
      PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL0[0], pcYuvDst.Y()),
        PelBuf(m_cRefSamplesDMVRL0[1], pcYuvDst.Cb()), PelBuf(m_cRefSamplesDMVRL0[2], pcYuvDst.Cr())));
    m_cYuvRefBuffDMVRL0 = m_cYuvRefBuffDMVRL0.subBuf(UnitAreaRelative(pu, subPu));

    // Inicializa os buffers de referência para a Lista 1 (L1) após a compensação de movimento DMVR
    m_cYuvRefBuffDMVRL1 = (pu.chromaFormat == CHROMA_400 ?
      PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL1[0], pcYuvDst.Y())) :
      PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL1[0], pcYuvDst.Y()), PelBuf(m_cRefSamplesDMVRL1[1], pcYuvDst.Cb()),
        PelBuf(m_cRefSamplesDMVRL1[2], pcYuvDst.Cr())));
    m_cYuvRefBuffDMVRL1 = m_cYuvRefBuffDMVRL1.subBuf(UnitAreaRelative(pu, subPu));
////////////////////////////////
/*
  - srcPred0 e srcPred1: São buffers de predição de referência para as Listas 0 (L0) e 1 (L1), respectivamente, após a compensação de movimento DMVR.
  - pu.chromaFormat: Verifica se o formato de croma é 400 (sem croma) ou não. Dependendo disso, são inicializados buffers diferentes para a componente de luminância e crominância.
  - PelUnitBuf e PelBuf: Representam estruturas de dados que contêm amostras de pixels. PelUnitBuf inclui amostras para todas as componentes (Y, Cb, Cr), enquanto PelBuf é específico para uma única componente.
  - m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]: São buffers de predição para a Lista 0 (L0) após DMVR, onde [0][0] é a luminância (Y), [0][1] é a crominância azul (Cb), e [0][2] é a crominância vermelha (Cr).
  - pcYuvDst.Y(), pcYuvDst.Cb(), pcYuvDst.Cr(): São os buffers de destino de amostras de pixels para a luminância (Y), crominância azul (Cb), e crominância vermelha (Cr) da unidade de predição, respectivamente.
  - subBuf(UnitAreaRelative(pu, subPu)): Atualiza os buffers de predição para a área específica da unidade de predição (pu), ajustando as posições e tamanhos de acordo.
*/
    // Inicializa os buffers de predição de referência para a Lista 0 (L0) após a compensação de movimento DMVR
    PelUnitBuf srcPred0 = (pu.chromaFormat == CHROMA_400 ?
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y())) :
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[0][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvDst.Cr())));
   
    // Inicializa os buffers de predição de referência para a Lista 1 (L1) após a compensação de movimento DMVR
    PelUnitBuf srcPred1 = (pu.chromaFormat == CHROMA_400 ?
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y())) :
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[1][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvDst.Cr())));

    // Atualiza os buffers de predição para a área específica da unidade de predição
    srcPred0 = srcPred0.subBuf(UnitAreaRelative(pu, subPu));
    srcPred1 = srcPred1.subBuf(UnitAreaRelative(pu, subPu));

////////////////////////////////
  /*
    O código realiza o cálculo de custo mínimo para diferentes deslocamentos de amostras de pixels com 
    compensação de movimento DMVR em uma unidade de predição. O custo é calculado comparando as amostras 
    preditas de Listas 0 e 1. O processo é repetido para diferentes iterações e posições dentro da unidade 
    de predição.
  */    
    int yStart = 0;
    for (int y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        // Cria uma subunidade de predição para a posição atual (x, y) com tamanho (dx, dy)
        PredictionUnit subPu = pu;
        subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
        
        // Pré-carrega os buffers de referência para as Listas 0 e 1
        xPrefetch(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0, 1);
        xPrefetch(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1, 1);

        // Inicializa os buffers de predição de referência para as Listas 0 e 1
        xinitMC(subPu, clpRngs);

        // Inicializa variáveis para o cálculo do custo mínimo
        uint64_t minCost = MAX_UINT64;
        bool notZeroCost = true;
        int16_t totalDeltaMV[2] = { 0,0 };
        int16_t deltaMV[2] = { 0, 0 };
        uint64_t  *pSADsArray;

        // Inicializa o array de SADs com valores máximos
        for (int i = 0; i < (((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)); i++)
        {
          m_SADsArray[i] = MAX_UINT64;
        }
        
        // Posiciona pSADsArray no meio do array m_SADsArray
        pSADsArray = &m_SADsArray[(((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)) >> 1];
        
        // Loop de iteração para cálculo de custo mínimo
        for (int i = 0; i < iterationCount; i++)
        {
          deltaMV[0] = 0;
          deltaMV[1] = 0;

          // Calcula endereços de predição com compensação de movimento para Listas 0 e 1
          Pel *addrL0 = biLinearPredL0 + totalDeltaMV[0] + (totalDeltaMV[1] * m_biLinearBufStride);
          Pel *addrL1 = biLinearPredL1 - totalDeltaMV[0] - (totalDeltaMV[1] * m_biLinearBufStride);
          
          // Cálculo do custo inicial usando a função xDMVRCost
          if (i == 0)
          {
            minCost = xDMVRCost(clpRngs.comp[COMPONENT_Y].bd, addrL0, m_biLinearBufStride, addrL1, m_biLinearBufStride, dx, dy);
            minCost -= (minCost >>2);
            
            // Verifica se o custo mínimo é menor que o número total de pixels
            if (minCost < (dx * dy))
            {
              notZeroCost = false;
              break;
            }
            pSADsArray[0] = minCost;
          }

          // Se o custo mínimo for zero, interrompe o loop
          if (!minCost)
          {
            notZeroCost = false;
            break;
          }
////////////////////////////////
          // Chama a função xBIPMVRefine para refinar o vetor de movimento com compensação de pixel
          xBIPMVRefine(bd, addrL0, addrL1, minCost, deltaMV, pSADsArray, dx, dy);

          // Verifica se o refinamento resultou em nenhum deslocamento, se sim, interrompe o loop
          if (deltaMV[0] == 0 && deltaMV[1] == 0)
          {
            break;
          }
          // Atualiza os deslocamentos acumulados com os valores resultantes do refinamento
          totalDeltaMV[0] += deltaMV[0];
          totalDeltaMV[1] += deltaMV[1];
          
          // Atualiza a posição no array de SADs com base nos deslocamentos
          pSADsArray += ((deltaMV[1] * (((2 * DMVR_NUM_ITERATION) + 1))) + deltaMV[0]);
        }
        
        // Determina se a técnica de bio é aplicada com base no custo mínimo em relação ao limiar definido (bioEnabledThres)
        bioAppliedType[num] = (minCost < bioEnabledThres) ? false : bioApplied;
        
        // Converte os deslocamentos acumulados para a escala de movimento usando o fator `mvShift`
        totalDeltaMV[0]     = totalDeltaMV[0] * (1 << mvShift);
        totalDeltaMV[1]     = totalDeltaMV[1] * (1 << mvShift);
        
        // Chama a função xDMVRSubPixelErrorSurface para processamento adicional, especialmente relacionado a erros subpixeis
        xDMVRSubPixelErrorSurface(notZeroCost, totalDeltaMV, deltaMV, pSADsArray);
        
        // Atualiza a estrutura de movimento para a subunidade de predição atual com os deslocamentos acumulados
        pu.mvdL0SubPu[num] = Mv(totalDeltaMV[0], totalDeltaMV[1]);

        // Obtém a subárea de predição dentro da imagem de destino para a subunidade de predição atual
        PelUnitBuf subPredBuf = pcYuvDst.subBuf(UnitAreaRelative(pu, subPu));
////////////////////////////////
        // Verifica se a subunidade de predição atual teve algum movimento
        bool blockMoved = false;
        if (pu.mvdL0SubPu[num] != Mv(0, 0))
        {
          blockMoved = true;

          // Prefetch das subunidades de predição para chroma, se habilitado
          if (isChromaEnabled(pu.chromaFormat))
          {
            xPrefetch(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0, 0);
            xPrefetch(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1, 0);
          }
          // Realiza o preenchimento (padding) nas subunidades de predição para L0 e L1
          xPad(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0);
          xPad(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1);
        }

        // Inicializa o array de strides para cada componente
        int dstStride[MAX_NUM_COMPONENT] = { pcYuvDst.bufs[COMPONENT_Y].stride,
                                             isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cb].stride : 0,
                                             isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cr].stride : 0};
        // Atualiza os vetores de movimento para a subunidade de predição
        subPu.mv[0] = mergeMv[REF_PIC_LIST_0] + pu.mvdL0SubPu[num];
        subPu.mv[1] = mergeMv[REF_PIC_LIST_1] - pu.mvdL0SubPu[num];
        
        // Realiza o clip dos vetores de movimento para a profundidade de bits de armazenamento
        subPu.mv[0].clipToStorageBitDepth();
        subPu.mv[1].clipToStorageBitDepth();

        // Chama a função xFinalPaddedMCForDMVR para realizar a compensação final para DMVR
        xFinalPaddedMCForDMVR(subPu, srcPred0, srcPred1, m_cYuvRefBuffDMVRL0, m_cYuvRefBuffDMVRL1, bioAppliedType[num],
                              mergeMv, blockMoved);

        // Atualiza o buffer de predição para apontar para a posição correta na imagem de destino
        subPredBuf.bufs[COMPONENT_Y].buf = pcYuvDst.bufs[COMPONENT_Y].buf + xStart + yStart * dstStride[COMPONENT_Y];

        // Atualiza os buffers de predição cromáticos, se habilitado
        if (isChromaEnabled(pu.chromaFormat))
        {
        subPredBuf.bufs[COMPONENT_Cb].buf = pcYuvDst.bufs[COMPONENT_Cb].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMPONENT_Cb]);
        subPredBuf.bufs[COMPONENT_Cr].buf = pcYuvDst.bufs[COMPONENT_Cr].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMPONENT_Cr]);
        }

        // Chama a função xWeightedAverage para realizar a média ponderada dos resultados
        xWeightedAverage(subPu, srcPred0, srcPred1, subPredBuf, subPu.cu->slice->getSPS()->getBitDepths(),
                         subPu.cu->slice->clpRngs(), bioAppliedType[num], false, false, nullptr);
        
        // Incrementa o contador de subunidades de predição processadas
        num++;
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(true);
}

////////////////////////////////

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
// Esta função atribui um objeto CacheModel à classe InterPrediction.
void InterPrediction::cacheAssign( CacheModel *cache )
{
  // Atribui o objeto cache à variável de membro m_cacheModel.
  m_cacheModel = cache;

  // Chama a função cacheAssign do objeto m_if, passando o mesmo objeto cache.
  m_if.cacheAssign( cache );

  // Inicializa o filtro de interpolação do objeto m_if, com base no estado do cache (ativado ou desativado).
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif
////////////////////////////////
/*
  Este código preenche o buffer IBC (Intra Block Copy) com os pixels reconstruídos da CodingUnit (CU). 
  Ele itera sobre todas as Transform Units (TUs) dentro da CU, copiando as áreas de componentes válidas 
  da CU para as posições correspondentes no buffer IBC. O buffer IBC é usado para armazenar informações 
  para o modo de predição Intra Block Copy.
*/
void InterPrediction::xFillIBCBuffer(CodingUnit &cu)
{
  // Itera sobre todas as TUs (Transform Units) da CodingUnit (cu)
  for (auto &currPU : CU::traverseTUs(cu))
  {
    // Itera sobre as áreas de componente (luma, croma) da PU
    for (const CompArea &area : currPU.blocks)
    {
      // Ignora áreas inválidas
      if (!area.valid())
      {
        continue;
      }

      // Obtém a largura máxima da CU em luma (unidade: amostra)
      const unsigned int lcuWidth = cu.cs->slice->getSPS()->getMaxCUWidth();
      // Obtém o fator de escala horizontal para a componente
      const int shiftSampleHor = ::getComponentScaleX(area.compID, cu.chromaFormat);
      // Obtém o fator de escala vertical para a componente
      const int shiftSampleVer = ::getComponentScaleY(area.compID, cu.chromaFormat);
      // Calcula o log2 do tamanho da CTU na vertical
      const int ctuSizeLog2Ver = floorLog2(lcuWidth) - shiftSampleVer;
      
      // Calcula as coordenadas dentro do bloco IBCBuffer
      const int pux = area.x & ((m_IBCBufferWidth >> shiftSampleHor) - 1);
      const int puy = area.y & (( 1 << ctuSizeLog2Ver ) - 1);
      // Cria uma área de destino (dst) com base nas coordenadas calculadas
      const CompArea dstArea = CompArea(area.compID, cu.chromaFormat, Position(pux, puy), Size(area.width, area.height));
      // Obtém um buffer de pixels de entrada (source) da CU
      CPelBuf srcBuf = cu.cs->getRecoBuf(area);
      // Obtém um buffer de pixels de destino (destination) do IBCBuffer
      PelBuf dstBuf = m_IBCBuffer.getBuf(dstArea);

      // Copia os pixels da área de origem (CU) para a área de destino (IBCBuffer)
      dstBuf.copyFrom(srcBuf);
    }
  }
}
////////////////////////////////
/*
  Este código realiza a predição Intra Block Copy (IBC) para uma dada Prediction Unit (PU) e componente específica (luma, croma). 
  Ele calcula as coordenadas de referência para a cópia de blocos IBC, considerando as diferenças de escala entre as componentes e 
  os fatores de deslocamento. O código copia os pixels correspondentes da área de referência do buffer IBC para o buffer de predição, 
  levando em consideração casos de wrap-around quando a cópia ultrapassa os limites do buffer.
*/
void InterPrediction::xIntraBlockCopy(PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID)
{
  // Obtém a largura máxima da CU em luma (unidade: amostra)
  const unsigned int lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
  // Obtém o fator de escala horizontal para a componente
  const int shiftSampleHor = ::getComponentScaleX(compID, pu.chromaFormat);
  // Obtém o fator de escala vertical para a componente
  const int shiftSampleVer = ::getComponentScaleY(compID, pu.chromaFormat);
  // Calcula o log2 do tamanho da CTU na vertical
  const int ctuSizeLog2Ver = floorLog2(lcuWidth) - shiftSampleVer;
  
  // Atribui o vetor de bloco (bv) da PU ao vetor de movimento da Lista 0 (L0)
  pu.bv = pu.mv[REF_PIC_LIST_0];
  pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  
  int refx, refy;
  // Calcula as coordenadas de referência para a predição IBC
  if (compID == COMPONENT_Y)
  {
    refx = pu.Y().x + pu.bv.hor;
    refy = pu.Y().y + pu.bv.ver;
  }
  else
  {//Cb or Cr
    refx = pu.Cb().x + (pu.bv.hor >> shiftSampleHor);
    refy = pu.Cb().y + (pu.bv.ver >> shiftSampleVer);
  }
  // Aplica a máscara para obter as coordenadas dentro do buffer IBC
  refx &= ((m_IBCBufferWidth >> shiftSampleHor) - 1);
  refy &= ((1 << ctuSizeLog2Ver) - 1);

  // Verifica se a cópia direta cabe dentro do buffer IBC
  if (refx + predBuf.bufs[compID].width <= (m_IBCBufferWidth >> shiftSampleHor))
  {
    // Área de origem dentro do buffer IBC
    const CompArea srcArea = CompArea(compID, pu.chromaFormat, Position(refx, refy), Size(predBuf.bufs[compID].width, predBuf.bufs[compID].height));
    // Obtém o buffer de referência IBC
    const CPelBuf refBuf = m_IBCBuffer.getBuf(srcArea);
    // Copia os pixels da área de referência para o buffer de predição   
    predBuf.bufs[compID].copyFrom(refBuf);
  }
  else
  {//wrap around
    int width = (m_IBCBufferWidth >> shiftSampleHor) - refx;
    
    // Área de origem na parte inicial do buffer IBC
    CompArea srcArea = CompArea(compID, pu.chromaFormat, Position(refx, refy), Size(width, predBuf.bufs[compID].height));
    CPelBuf srcBuf = m_IBCBuffer.getBuf(srcArea);
    PelBuf dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position(0, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);

    width = refx + predBuf.bufs[compID].width - (m_IBCBufferWidth >> shiftSampleHor);

    // Área de origem na parte final do buffer IBC
    srcArea = CompArea(compID, pu.chromaFormat, Position(0, refy), Size(width, predBuf.bufs[compID].height));
    srcBuf = m_IBCBuffer.getBuf(srcArea);
    dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position((m_IBCBufferWidth >> shiftSampleHor) - refx, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);
  }
}

////////////////////////////////////////////////////////////////
/*
  Este código reinicializa o buffer IBC (Intra Block Copy) para uma determinada Chroma Format e 
  tamanho do bloco CTU (Coding Tree Unit). A função preenche todo o buffer IBC com o valor -1. 
  Isso pode ser útil para indicar que o buffer está vazio ou não contém dados válidos.
*/
void InterPrediction::resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize)
{
  // Cria uma área correspondente ao tamanho do bloco CTU
  const UnitArea area = UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize));
  // Preenche o buffer IBC com o valor -1
  m_IBCBuffer.getBuf(area).fill(-1);
}
////////////////////////////////////////////////////////////////
/*
  Esta função é responsável por resetar uma parte específica do buffer IBC (Intra Block Copy) 
  associada a uma VPDU (Variable-sized Prediction Data Unit). A VPDU é especificada pela sua 
  posição (xPos, yPos) e seu tamanho (vSize). O reset é feito preenchendo a região correspondente 
  com o valor -1. Isso pode ser útil para limpar uma parte do buffer que não contém dados válidos 
  ou precisa ser reinicializada.
*/
void InterPrediction::resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos)
{
  // Cria uma área correspondente à posição (xPos, yPos) dentro do buffer IBC
  const UnitArea area = UnitArea(chromaFormatIDC, Area(xPos & (m_IBCBufferWidth - 1), yPos & (ctuSize - 1), vSize, vSize));
  // Preenche a região da VPDU (Variable-sized Prediction Data Unit) no buffer IBC com o valor -1
  m_IBCBuffer.getBuf(area).fill(-1);
}

////////////////////////////////////////////////////////////////
/*
  Essa função verifica se uma VPDU (Variable-sized Prediction Data Unit) na componente de luminância 
  é válida no buffer IBC (Intra Block Copy). Ela realiza essa verificação ao iterar sobre os blocos 4x4 
  na VPDU e checar se todos os pixels têm valores válidos no buffer IBC. A função retorna verdadeiro se 
/ todos os pixels na VPDU são válidos e falso caso contrário.
*/
bool InterPrediction::isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv)
{
  // Verifica se a VPDU (Variable-sized Prediction Data Unit) ultrapassa os limites do bloco CTU (Coding Tree Unit)
  if(((yCb + yBv) & (ctuSize - 1)) + height > ctuSize)
  {
    return false;
  }
  // Calcula as coordenadas do bloco de referência no canto superior esquerdo
  int refTLx = xCb + xBv;
  int refTLy = (yCb + yBv) & (ctuSize - 1);
  
  // Obtém o buffer de luminância do buffer IBC
  PelBuf buf = m_IBCBuffer.Y();
  
  // Itera sobre blocos de 4x4 na VPDU
  for(int x = 0; x < width; x += 4)
  {
    for(int y = 0; y < height; y += 4)
    {
      // Verifica se todos os pixels no bloco 4x4 têm valores válidos no buffer IBC
      if(buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if(buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if(buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
      if(buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
    }
  }
  // Todos os pixels na VPDU são válidos no buffer IBC
  return true;
}

////////////////////////////////////////////////////////////////
/*
  Essa função realiza a predição de um bloco usando o método de Refineable Picture Resampling (RPR).
  A função recebe parâmetros como razão de escala, informações sobre o bloco de predição, a imagem de referência, o vetor 
  de movimento, o destino da predição, entre outros. A variável rndRes indica se o resultado deve ser arredondado, sendo 
  falso para predição bidirecional e verdadeiro para predição unidirecional. A quantidade de bits para deslocamento hori-
  zontal e vertical é configurada com base na componente do bloco (luma ou croma). O código continua, mas detalhes especí-
  ficos sobre o método de RPR e o restante da função não são fornecidos aqui.
*/

bool InterPrediction::xPredInterBlkRPR(const std::pair<int, int> &scalingRatio, const PPS &pps, const CompArea &blk,
                                       const Picture *refPic, const Mv &mv, Pel *dst, const int dstStride,
                                       const bool bi, const bool wrapRef, const ClpRng &clpRng,
                                       const InterpolationFilter::Filter filterIndex, const bool useAltHpelIf)
{
  // Obtém as propriedades do bloco de referência
  const ChromaFormat  chFmt = blk.chromaFormat;
  const ComponentID compID = blk.compID;
  const bool          rndRes = !bi;     // Se é uma predição unidirecional, o resultado não é arredondado

  // Configura a quantidade de bits para deslocamento horizontal e vertical, considerando a componente
  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + (isLuma(compID) ? 0 : 1);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + (isLuma(compID) ? 0 : 1);

  // Obtém as dimensões do bloco
  int width = blk.width;
  int height = blk.height;
  CPelBuf refBuf;

  // Verifica se a referência foi escalada
  const bool scaled = refPic->isRefScaled( &pps );
////////////////////////////////
  if( scaled )
  {
    // Obtém as dimensões da imagem de referência
    int row, col;
    int refPicWidth = refPic->getPicWidthInLumaSamples();
    int refPicHeight = refPic->getPicHeightInLumaSamples();
    
    // Configura os filtros de interpolação para RPR(Refineable Picture Resampling) com base nas razões de escala
    InterpolationFilter::Filter xFilter       = filterIndex;
    InterpolationFilter::Filter yFilter       = filterIndex;
    const int rprThreshold1 = ( 1 << SCALE_RATIO_BITS ) * 5 / 4;
    const int rprThreshold2 = ( 1 << SCALE_RATIO_BITS ) * 7 / 4;
    if (filterIndex == InterpolationFilter::Filter::DEFAULT || !isLuma(compID))
    {
      if( scalingRatio.first > rprThreshold2 )
      {
        xFilter = InterpolationFilter::Filter::RPR2;
      }
      else if( scalingRatio.first > rprThreshold1 )
      {
        xFilter = InterpolationFilter::Filter::RPR1;
      }

      if( scalingRatio.second > rprThreshold2 )
      {
        yFilter = InterpolationFilter::Filter::RPR2;
      }
      else if( scalingRatio.second > rprThreshold1 )
      {
        yFilter = InterpolationFilter::Filter::RPR1;
      }
    }
    else if (filterIndex == InterpolationFilter::Filter::AFFINE)
    {
      if (scalingRatio.first > rprThreshold2)
      {
        xFilter = InterpolationFilter::Filter::AFFINE_RPR2;
      }
      else if (scalingRatio.first > rprThreshold1)
      {
        xFilter = InterpolationFilter::Filter::AFFINE_RPR1;
      }

      if (scalingRatio.second > rprThreshold2)
      {
        yFilter = InterpolationFilter::Filter::AFFINE_RPR2;
      }
      else if (scalingRatio.second > rprThreshold1)
      {
        yFilter = InterpolationFilter::Filter::AFFINE_RPR1;
      }
    }

    // Configura filtros de interpolação alternativos se necessário
    if (useAltHpelIf)
    {
      if (xFilter == InterpolationFilter::Filter::DEFAULT && scalingRatio.first == 1 << SCALE_RATIO_BITS)
      {
        xFilter = InterpolationFilter::Filter::HALFPEL_ALT;
      }
      if (yFilter == InterpolationFilter::Filter::DEFAULT && scalingRatio.second == 1 << SCALE_RATIO_BITS)
      {
        yFilter = InterpolationFilter::Filter::HALFPEL_ALT;
      }
    }
    // Configurações adicionais para interpolação RPR
    const int posShift = SCALE_RATIO_BITS - 4;

    const int stepX = (scalingRatio.first + 8) >> 4;
    const int stepY = (scalingRatio.second + 8) >> 4;

    const int offX = 1 << (posShift - shiftHor - 1);
    const int offY = 1 << (posShift - shiftVer - 1);

    // Obtém informações sobre a crominância
    const uint32_t scaleX = ::getComponentScaleX(compID, chFmt);
    const uint32_t scaleY = ::getComponentScaleY(compID, chFmt);

    // Calcula a posição ajustada
    const int64_t posX = ((blk.pos().x << scaleX) - (pps.getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX(chFmt))) >> scaleX;
    const int64_t posY = ((blk.pos().y << scaleY) - (pps.getScalingWindow().getWindowTopOffset() * SPS::getWinUnitY(chFmt))) >> scaleY;

    //Ajusta adicionando offsets(compensações)
    int addX = isLuma( compID ) ? 0 : int( 1 - refPic->cs->sps->getHorCollocatedChromaFlag() ) * 8 * ( scalingRatio.first - SCALE_1X.first );
    int addY = isLuma( compID ) ? 0 : int( 1 - refPic->cs->sps->getVerCollocatedChromaFlag() ) * 8 * ( scalingRatio.second - SCALE_1X.second );
////////////////////////////////
    // Define as margens para limitar a área de busca na imagem de referência
    int boundLeft   = 0;
    int boundRight  = refPicWidth >> scaleX;
    int boundTop    = 0;
    int boundBottom = refPicHeight >> scaleY;
    // Verifica se há várias sub-imagens na imagem de referência
    if( refPic->subPictures.size() > 1 )
    {
      // Obtém informações sobre a sub-imagem atual
      const SubPic& curSubPic = pps.getSubPicFromPos(blk.lumaPos());
      // Atualiza as margens se a sub-imagem atual for tratada como uma imagem completa
      if( curSubPic.getTreatedAsPicFlag() )
      {
        boundLeft   = curSubPic.getSubPicLeft() >> scaleX;
        boundRight  = curSubPic.getSubPicRight() >> scaleX;
        boundTop    = curSubPic.getSubPicTop() >> scaleY;
        boundBottom = curSubPic.getSubPicBottom() >> scaleY;
      }
    }
///////////////////////////////////
    int64_t x0Int;
    int64_t y0Int;
    
    // Calcula as posições iniciais em x e y para interpolação RPR
    x0Int = ((posX << (4 + scaleX)) + mv.getHor()) * (int64_t) scalingRatio.first + addX;
    x0Int = sgn2(x0Int) * ((abs(x0Int) + (1ull << (7 + scaleX))) >> (8 + scaleX))
            + ((refPic->getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX(chFmt)) << ((posShift - scaleX)));

    y0Int = ((posY << (4 + scaleY)) + mv.getVer()) * (int64_t) scalingRatio.second + addY;
    y0Int = sgn2(y0Int) * ((abs(y0Int) + (1ull << (7 + scaleY))) >> (8 + scaleY))
            + ((refPic->getScalingWindow().getWindowTopOffset() * SPS::getWinUnitY(chFmt)) << ((posShift - scaleY)));

    // Define o tamanho do filtro e o número de taps
    const int extSize = isLuma( compID ) ? 1 : 2;
    int vFilterSize = isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

    // Calcula a posição inicial em y e limita dentro da imagem de referência
    int yInt0 = ( (int32_t)y0Int + offY ) >> posShift;
    yInt0 = std::min( std::max( boundTop - (NTAPS_LUMA / 2), yInt0 ), boundBottom + (NTAPS_LUMA / 2) );
    
    // Calcula a posição inicial em x e limita dentro da imagem de referência
    int xInt0 = ( (int32_t)x0Int + offX ) >> posShift;
    xInt0 = std::min( std::max( boundLeft - (NTAPS_LUMA / 2), xInt0 ), boundRight + (NTAPS_LUMA / 2) );

    // Calcula a altura da área de referência
    int refHeight = ((((int32_t)y0Int + (height-1) * stepY) + offY ) >> posShift) - ((((int32_t)y0Int + 0 * stepY) + offY ) >> posShift) + 1;
    refHeight = std::max<int>( 1, refHeight );

    // Verifica se o buffer temporário é grande o suficiente
    CHECK(TMP_RPR_HEIGHT < refHeight + vFilterSize - 1 + extSize,
          "Buffer is not large enough, increase MAX_SCALING_RATIO");

    // Inicializa variáveis
    int tmpStride = width;
    int xInt = 0, yInt = 0;

    // Loop de aplicação do filtro horizontal
    for( col = 0; col < width; col++ )
    {
      // Calcula a posição em x para a interpolação RPR
      int posX = (int32_t)x0Int + col * stepX;
      xInt = ( posX + offX ) >> posShift;
      xInt = std::min( std::max( boundLeft - (NTAPS_LUMA / 2), xInt ), boundRight + (NTAPS_LUMA / 2) );
      int xFrac = ( ( posX + offX ) >> ( posShift - shiftHor ) ) & ( ( 1 << shiftHor ) - 1 );

      // Verifica a posição inicial em x
      CHECK( xInt0 > xInt, "Wrong horizontal starting point" );

      // Obtém a área de referência para o filtro horizontal
      Position offset = Position( xInt, yInt0 );
      refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, Size( 1, refHeight ) ), wrapRef );

      Pel *const tempBuf = m_filteredBlockTmpRPR + col;

      // Aplica o filtro horizontal
      m_if.filterHor(compID, (Pel *) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tempBuf,
                     tmpStride, 1, refHeight + vFilterSize - 1 + extSize, xFrac, false, clpRng, xFilter);
    }

    // Loop de aplicação do filtro vertical
    for( row = 0; row < height; row++ )
    {
      // Calcula a posição em y para a interpolação RPR
      int posY = (int32_t)y0Int + row * stepY;
      yInt = ( posY + offY ) >> posShift;
      yInt = std::min( std::max( boundTop - (NTAPS_LUMA / 2), yInt ), boundBottom + (NTAPS_LUMA / 2) );
      int yFrac = ( ( posY + offY ) >> ( posShift - shiftVer ) ) & ( ( 1 << shiftVer ) - 1 );

      // Verifica a posição inicial em y
      CHECK( yInt0 > yInt, "Wrong vertical starting point" );

      const Pel *const tempBuf = m_filteredBlockTmpRPR + (yInt - yInt0) * tmpStride;

      // Aplica o filtro vertical
      JVET_J0090_SET_CACHE_ENABLE( false );
      m_if.filterVer(compID, tempBuf + ((vFilterSize >> 1) - 1) * tmpStride, tmpStride, dst + row * dstStride,
                     dstStride, width, 1, yFrac, false, rndRes, clpRng, yFilter);
      JVET_J0090_SET_CACHE_ENABLE( true );
    }
  }
  // Retorna se a interpolação RPR foi escalada
  return scaled;
}

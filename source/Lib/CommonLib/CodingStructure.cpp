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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"

// Inicialização da XUCache global
XUCache g_globalUnitCache = XUCache();

// Definicação da matriz de escala de unidade para diferentes formatos cromáticos e componentes
const UnitScale UnitScaleArray[NUM_CHROMA_FORMAT][MAX_NUM_COMPONENT] =
{
  { {2,2}, {0,0}, {0,0} },  // 4:0:0
  { {2,2}, {1,1}, {1,1} },  // 4:2:0
  { {2,2}, {1,2}, {1,2} },  // 4:2:2
  { {2,2}, {2,2}, {2,2} }   // 4:4:4
};

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------
// Construtor da classe CodingStructure
CodingStructure::CodingStructure(CUCache& cuCache, PUCache& puCache, TUCache& tuCache)
  : area      ()
  , picture   ( nullptr )
  , parent    ( nullptr )
  , bestCS    ( nullptr )
  , m_isTuEnc ( false )
  , m_cuCache ( cuCache )
  , m_puCache ( puCache )
  , m_tuCache ( tuCache )
  , bestParent ( nullptr )
  , tmpColorSpaceCost(MAX_DOUBLE)
  , firstColorSpaceSelected(true)
  , resetIBCBuffer (false)
{ // Inicialização dos membros de array e ponteiros para coeficientes e buffers
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_coeffs[ i ] = nullptr;
    m_pcmbuf[ i ] = nullptr;
    m_offsets[ i ] = 0;
  }
  // Inicialização dos ponteiros para tipos de **corrida(?)**
  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_runType[i] = nullptr;
  }
  // Inicialização dos ponteiros e flags para índices, decomposição e buffers de movimento
  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_cuIdx   [ i ] = nullptr;
    m_puIdx   [ i ] = nullptr;
    m_tuIdx   [ i ] = nullptr;
    m_isDecomp[ i ] = nullptr;
  }
  // Inicialição do buffer de movimento
  m_motionBuf     = nullptr;
// Inicialição do cabeçalho da imagem (apenas se GDR estiver habilitado)
#if GDR_ENABLED
  picHeader = nullptr;
#endif
  // Inicialização de variáveis relacionadas a recursos de codificação
  features.resize( NUM_ENC_FEATURES );
  treeType = TREE_D;
  modeType = MODE_TYPE_ALL;
  tmpColorSpaceIntraCost[0] = MAX_DOUBLE;
  tmpColorSpaceIntraCost[1] = MAX_DOUBLE;
  firstColorSpaceTestOnly = false;
}

void CodingStructure::destroy()
{ //Desassociando ponteiros
  picture   = nullptr;
  parent    = nullptr;
  // Chamando a função destroy() para as estruturas m_pred, m_resi, m_reco e m_orgr
  m_pred.destroy();
  m_resi.destroy();
  m_reco.destroy();
  m_orgr.destroy();
  // Liberando os coeficientes
  destroyCoeffs();

#if GDR_ENABLED
  // Verificando se há um picHeader e se o GDR está habilitado
  if (picHeader && m_gdrEnabled)
  {
    delete picHeader; // Liberando picHeader se GDR habilitado
  }

  picHeader = nullptr;
#endif
  // Liberando memória alocada dinamicamente para m_isDecomp, m_cuIdx, m_puIdx, m_tuIdx
  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    delete[] m_isDecomp[ i ];
    m_isDecomp[ i ] = nullptr;

    delete[] m_cuIdx[ i ];
    m_cuIdx[ i ] = nullptr;

    delete[] m_puIdx[ i ];
    m_puIdx[ i ] = nullptr;

    delete[] m_tuIdx[ i ];
    m_tuIdx[ i ] = nullptr;
  }
  // Liberando o m_motionBuf
  delete[] m_motionBuf;
  m_motionBuf = nullptr;

  // Retornando as unidades de transformação (tus), predição (pus) e codificação (cus) para o cache
  m_tuCache.cache( tus );
  m_puCache.cache( pus );
  m_cuCache.cache( cus );
}

void CodingStructure::releaseIntermediateData()
{   // Chama as funções clearTUs(), clearPUs() e clearCUs() para liberar dados intermediários
  clearTUs();
  clearPUs();
  clearCUs();
}

#if GDR_ENABLED
bool CodingStructure::containRefresh(int begX, int endX) const
{ // Verifica se begX é igual a endX, indicando uma área vazia.
  if (begX == endX)
  {
    return false;   // se for uma área vazia, retorna false
  }
  // Obtém a área total da CodingStructure na componente de luminância (Y).
  const Area csArea      = area.Y();
  // Cria uma área de atualização usando os parâmetros begX, endX e informações da CodingStructure.
  const Area refreshArea = Area(begX, area.ly(), endX - begX, std::min(slice->getPPS()->getPicHeightInLumaSamples(), area.lheight()));
  // Verifica se a área de atualização está contida dentro da área total da CodingStructure.
  if (csArea.contains(refreshArea))
  {
    return true; // Se estiver contida, retorna true, indicando que a área de atualização está presente.
  }

  return false;  // Se a área de atualização não estiver contida, retorna false.
}

/*
  As próximas duas funções em CodingStructure são usadas para determinar se há uma sobreposição entre a área de atualização 
e a área total da CodingStructure.
*/
bool CodingStructure::overlapRefresh(int begX, int endX) const
{
  if (begX == endX)   // Verifica se begX é igual a endX, indicando uma área vazia.
  {
    return false;     // Não há sobreposição em uma área vazia
  }
  // Obtém a área total da CodingStructure na componente de luminância (Y).
  const Area csArea = area.Y();
  // Cria uma área de atualização usando os parâmetros begX e endX.
  const Area refreshArea = Area(begX, area.ly(), endX - begX, area.lheight());
  // Verifica se há sobreposição entre a área total e a área de atualização.
  if (csArea.overlap(refreshArea))
  {
    return true;      // Há sobreposição
  }

  return false;       // Não há sobreposição
}

bool CodingStructure::overlapRefresh() const
{
  // Obtém a coordenada X (canto superior esquerdo) e a largura da CodingStructure na componente de luminância (Y).
  const int  csX     = area.lx();
  const int  csWidth = area.lwidth();
  // Chama overlapRefresh() com as coordenadas X e X + largura para verificar a sobreposição.
  bool ret = overlapRefresh(csX, csX + csWidth);

  return ret; // retorna o resultado
}

bool CodingStructure::withinRefresh(int begX, int endX) const
{
  if (begX == endX)   // Verifica se begX é igual a endX, indicando uma área vazia
  {
    return false;     // não está dentro de uma área vazia
  }
  // Obtém a área total da CodingStructure no componente de luminância (Y)
  const Area csArea = area.Y();
  // Cria uma área de atualização usando os parâmetros begX e endX
  const Area refreshArea = Area(begX, area.ly(), endX - begX, area.lheight());
  // Verifica se a área total da CodingStructure está completamente contida na área de atualização
  if (refreshArea.contains(csArea))
  {
    return true;  // Está completamente contida
  }

  return false;   // Não está completamente contida
}

/*
  A função CodingStructure::refreshCrossTTV(int begX, int endX) const verifica se existe uma sobreposição entre uma área de atualização 
  específica (refreshArea) e três áreas de codificação (csArea0, csArea1 e csArea2) divididas horizontalmente na componente de luminância 
  (Y) da CodingStructure.
*/
bool CodingStructure::refreshCrossTTV(int begX, int endX) const
{  // Obtém as coordenadas e dimensões da CodingStructure na componente de luminância (Y).
  const int  csX = area.lx();
  const int  csY = area.ly();
  const int  csWidth  = area.lwidth();
  const int  csHeight = area.lheight();
  // Cria uma área de atualização usando os parâmetros begX e endX.
  const Area refreshArea = Area(begX, csY, endX - begX, csHeight);
  // Divide a CodingStructure em três áreas horizontais.
  const Area csArea0 = Area(csX,                                   csY, csWidth >> 2, csHeight);
  const Area csArea1 = Area(csX + (csWidth >> 2),                  csY, csWidth >> 1, csHeight);
  const Area csArea2 = Area(csX + (csWidth >> 2) + (csWidth >> 1), csY, csWidth >> 2, csHeight);
  // Verifica se há sobreposição entre cada área de codificação e a área de atualização.
  bool overlap0 = csArea0.overlap(refreshArea);
  bool overlap1 = csArea1.overlap(refreshArea);
  bool overlap2 = csArea2.overlap(refreshArea);
  // Soma o número de sobreposições.
  int sum = (overlap0 ? 1 : 0) + (overlap1 ? 1 : 0) + (overlap2 ? 1 : 0);
  // Se houver pelo menos uma sobreposição, retorna true; caso contrário, retorna false.
  if (0 < sum)
  {
    return true;  //  existe sobreposição
  }

  return false;   // não há sobreposição
}

/*
  A função CodingStructure::refreshCrossBTV(int begX, int endX) const verifica se existe uma sobreposição entre uma área de atualização 
  específica (refreshArea) e duas áreas de codificação horizontalmente divididas (csArea0 e csArea1) na componente de luminância (Y) da 
  CodingStructure.
*/
bool CodingStructure::refreshCrossBTV(int begX, int endX) const
{
  // Obtém as coordenadas e dimensões da CodingStructure na componente de luminância (Y).
  const int  csX = area.lx();
  const int  csY = area.ly();
  const int  csWidth = area.lwidth();
  const int  csHeight = area.lheight();
  // Cria uma área de atualização usando os parâmetros begX e endX.
  const Area refreshArea = Area(begX, csY, endX - begX, csHeight);
  // Divide a CodingStructure em duas áreas horizontais.
  const Area csArea0 = Area(csX,                  csY, (csWidth >> 1), csHeight);
  const Area csArea1 = Area(csX + (csWidth >> 1), csY, (csWidth >> 1), csHeight);
  // Verifica se há sobreposição entre cada área de codificação e a área de atualização.
  bool overlap0 = csArea0.overlap(refreshArea);
  bool overlap1 = csArea1.overlap(refreshArea);
  // Soma o número de sobreposições.
  int sum = (overlap0 ? 1 : 0) + (overlap1 ? 1 : 0);
  // Se houver pelo menos uma sobreposição, retorna true; caso contrário, retorna false.
  if (0 < sum)
  {
    return true;  //  existe sobreposição
  }

  return false;   // não há sobreposição
}

/*
  A função CodingStructure::overlapDirty() verifica se há uma condição onde uma parte da imagem (luma) está marcada como "não limpa" 
  (isClean retorna false), enquanto outra parte da imagem (luma) está marcada como "limpa" (isClean retorna true).
*/
bool CodingStructure::overlapDirty() const
{
  // Obtém as posições do canto superior esquerdo e superior direito da área de luma.
  const Position topLeft  = area.Y().topLeft();
  const Position topRight = area.Y().topRight();
  // Verifica se a área à esquerda é considerada "limpa" ou "não limpa".
  bool insideLeft  = isClean(topLeft, CHANNEL_TYPE_LUMA);
  // Verifica se a área à direita é considerada "limpa" ou "não limpa".
  bool insideRight = isClean(topRight, CHANNEL_TYPE_LUMA);
  // Se a condição de limpeza à esquerda é diferente da condição à direita, há sobreposição "suja".
  if (insideLeft != insideRight)
  {
    return true;    // há sopreposição "suja"
  }

  return false;     // não há sopreposição "suja"
}

bool CodingStructure::dirtyCrossTTV() const
{
  const int  csX = area.lx();
  const int  csY = area.ly();
  const int  csWidth = area.lwidth();
  const int  csHeight = area.lheight();
  // Divide a CodingStructure em três áreas horizontais.
  const Area csArea0 = Area(csX, csY, csWidth >> 2, csHeight);
  const Area csArea1 = Area(csX + (csWidth >> 2), csY, csWidth >> 1, csHeight);
  const Area csArea2 = Area(csX + (csWidth >> 2) + (csWidth >> 1), csY, csWidth >> 2, csHeight);
  // Verifica se cada uma das três áreas está marcada como "limpa" ou "não limpa".
  bool clean0 = isClean(csArea0, CHANNEL_TYPE_LUMA);
  bool clean1 = isClean(csArea1, CHANNEL_TYPE_LUMA);
  bool clean2 = isClean(csArea2, CHANNEL_TYPE_LUMA);
  // Verifica se todas as três áreas estão limpas.
  bool allclean = clean0 && clean1 && clean2;
  // Se todas as três áreas estão limpas, retorna false; caso contrário, retorna true.
  if (allclean)
  {
    return false;   // não há sujeira
  }

  return true;      // há sujeira
}

bool CodingStructure::dirtyCrossBTV() const
{
  const int  csX = area.lx();
  const int  csY = area.ly();
  const int  csWidth = area.lwidth();
  const int  csHeight = area.lheight();
  // Divide a CodingStructure em duas áreas horizontais.
  const Area csArea0 = Area(csX,                  csY, (csWidth >> 1), csHeight);
  const Area csArea1 = Area(csX + (csWidth >> 1), csY, (csWidth >> 1), csHeight);
  // Verifica se cada uma das duas áreas está marcada como "limpa" ou "não limpa".
  bool clean0 = isClean(csArea0, CHANNEL_TYPE_LUMA);
  bool clean1 = isClean(csArea1, CHANNEL_TYPE_LUMA);
  // Verifica se ambas as áreas estão limpas.
  bool allclean = clean0 && clean1;
  // Se ambas as áreas estão limpas, retorna false; caso contrário, retorna true.
  if (allclean)
  {
    return false;   // não há sujeira
  }

  return true;      // há sujeira
}
#endif



#if GDR_ENABLED
/*
  A função CodingStructure::isClean(const Position &IntPos, Mv FracMv) const verifica se uma determinada posição, especificada por IntPos e FracMv, 
  está localizada em uma área "limpa" ou "suja" em uma imagem durante um intervalo de GDR (Gradual Decoder Refresh).
*/
bool CodingStructure::isClean(const Position &IntPos, Mv FracMv) const
{
  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */
  const Picture* const curPic = slice->getPic();

  if (!curPic)
  {
    return false;
  }

  PicHeader     *curPh = curPic->cs->picHeader;
  if (!curPh)
  {
    return false;
  }
  bool isCurGdrPicture = curPh->getInGdrInterval();

  if (isCurGdrPicture)    // 2. Se a imagem atual é uma imagem GDR:
  {
    // 3. Define as distâncias de pixels para a verificação de sujeira.
    const int lumaPixelAway = 4;
    const int chromaPixelAway = 5;

    // 4. Configurações de deslocamento para manipulação de Mv.
    const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
    const int iMvLumaFrac = (1 << iMvShift);
    const int iMvChromaFrac = (iMvLumaFrac << 1);
    
    // 5. Verifica se as componentes horizontais de Mv são números inteiros.
    const bool isIntLumaMv = (FracMv.getHor() % iMvLumaFrac) == 0;
    const bool isIntChromaMv = (FracMv.getHor() % iMvChromaFrac) == 0;
    // 6. Compara a posição da última amostra com a posição de fim virtual da imagem.
    const int scaledEndX = curPh->getVirtualBoundariesPosX(0) << iMvShift;

    // 7. Calcula a posição da última amostra (último pixel) ao longo do eixo horizontal.
    const Position OrigFracPos = Position(IntPos.x << iMvShift, IntPos.y << iMvShift);
    const int lastLumaPos = ((OrigFracPos.x / iMvLumaFrac)   * iMvLumaFrac) + FracMv.getHor() + (isIntLumaMv ? 0 : (lumaPixelAway << iMvShift));
    const int lastChromaPos = ((OrigFracPos.x / iMvChromaFrac) * iMvChromaFrac) + FracMv.getHor() + (isIntChromaMv ? 0 : (chromaPixelAway << iMvShift));
    // 8. Determina a posição da última amostra.
    const int lastPelPos = std::max(lastLumaPos, lastChromaPos);
    // 9. Se a posição da última amostra é menor que a posição final virtual, retorna true (área limpa); caso contrário, retorna false (área suja).
    if (lastPelPos < scaledEndX)
    {
      return true;      // área limpa
    }
    else
    {
      return false;     // área suja
    }
  }

  return true;          // 10. Se não é uma imagem GDR, considera a posição como "limpa".
}

/*
  A função CodingStructure::isClean realiza uma verificação semelhante à anterior, mas agora ela compara a posição de um bloco em relação à sua 
  posição em uma imagem de referência (refPic). A lógica é utilizada para determinar se o bloco está em uma área "limpa" ou "suja" durante o 
  processo de Gradual Decoder Refresh (GDR).
*/
bool CodingStructure::isClean(const Position &IntPos, const Mv FracMv, const Picture *const refPic) const
{
  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */
  if (!refPic)        // Verifica se não há uma imagem de referência.
  {
    return false;
  }

  if (!refPic->cs)    // Verifica se a imagem de referência não possui uma estrutura de codificação (CodingStructure).
  {
    return false;
  }

  PicHeader *refPh = refPic->cs->picHeader;   // Obtém o cabeçalho da imagem de referência.
  if (!refPh)         // Verifica se há um cabeçalho de imagem de referência.
  {
    return false;
  }
  // Verifica se a imagem de referência está em um intervalo GDR (Gradual Decoder Refresh).
  bool isRefGdrPicture = refPh->getInGdrInterval();

  if (isRefGdrPicture)
  { 
    // Configurações específicas para o GDR.
    const int lumaPixelAway = 4;
    const int chromaPixelAway = 5;

    const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
    const int iMvLumaFrac = (1 << iMvShift);
    const int iMvChromaFrac = (iMvLumaFrac << 1);

    const bool isIntLumaMv = (FracMv.getHor() % iMvLumaFrac) == 0;
    const bool isIntChromaMv = (FracMv.getHor() % iMvChromaFrac) == 0;

    const int  scaledEndX = refPh->getVirtualBoundariesPosX(0) << iMvShift;
    // Calcula a posição do último pixel em relação ao movimento fracionário.
    const Position OrigFracPos = Position((IntPos.x) << iMvShift, IntPos.y << iMvShift);
    const int lastLumaPos = ((OrigFracPos.x / iMvLumaFrac)   * iMvLumaFrac) + FracMv.getHor() + (isIntLumaMv ? 0 : (lumaPixelAway << iMvShift));
    const int lastChromaPos = ((OrigFracPos.x / iMvChromaFrac) * iMvChromaFrac) + FracMv.getHor() + (isIntChromaMv ? 0 : (chromaPixelAway << iMvShift));
    // Determina a posição do último pixel entre a posição luma e croma.
    const int lastPelPos = std::max(lastLumaPos, lastChromaPos);
    // Verifica se a posição do último pixel está dentro da área limpa.
    if (lastPelPos < scaledEndX)
    {
      return true;    // está em uma área limpa
    }
    else
    {
      return false;  // está em uma área suja
    }
  }
  else
  {
    // refPic is normal picture
    // Se a imagem de referência não está em um intervalo GDR, verifica se a imagem
    // atual (referência) está em um intervalo GDR.
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);

    if (isCurGdrPicture)
    {
      return false;   //  está em uma área suja
    }
    else
    {
      return true;    // está em uma área limpa
    }
  }
}


bool CodingStructure::isClean(const Position &IntPos, Mv FracMv, RefPicList e, int refIdx, int isProf) const
{
  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */  // Verifica se o índice de referência é válido e obtém a imagem de referência correspondente
  if (refIdx < 0)
  {
    return false;
  }

  const Picture* const refPic = slice->getRefPic(e, refIdx);
  const bool isExceedNumRef = (refIdx < slice->getNumRefIdx(e)) ? false : true;

  if (!refPic || isExceedNumRef)
  {
    return false;
  }
  // Verifica se a imagem de referência possui um CodingStructure válido
  if (!refPic->cs)
  {
    return false;
  }
  // Obtém o cabeçalho da imagem de referência
  PicHeader *refPh = refPic->cs->picHeader;

  if (!refPh)
  {
    return false;
  }
  // Verifica se a imagem de referência está dentro de um intervalo GDR (Gradual Decoder Refresh)
  bool isRefGdrPicture = refPh->getInGdrInterval();

  if (isRefGdrPicture)  // Se a imagem de referência está dentro de um intervalo GDR
  { 
    // Ajustes para verificar se a posição é limpa ou suja com base no GDR
    const int lumaPixelAway   = 4 + (isProf << 0);
    const int chromaPixelAway = 4 + (isProf << 1);

    const int iMvShift      = MV_FRACTIONAL_BITS_INTERNAL;
    const int iMvLumaFrac   = (1 << iMvShift);
    const int iMvChromaFrac = (iMvLumaFrac << 1);

    const bool isIntLumaMv      = (FracMv.getHor() % iMvLumaFrac  ) == 0;
    const bool isIntChromaMv    = isProf ? false : (FracMv.getHor() % iMvChromaFrac) == 0;

    const int  scaledEndX      = refPh->getVirtualBoundariesPosX(0) << iMvShift;

    // Posição original ajustada para o GDR
    const Position OrigFracPos  = Position((IntPos.x) << iMvShift, IntPos.y << iMvShift);
    // Última posição luma e croma considerando o GDR
    const int lastLumaPos     = ((OrigFracPos.x / iMvLumaFrac)   * iMvLumaFrac)   + FracMv.getHor() + (isIntLumaMv   ? 0 : (lumaPixelAway   << iMvShift));
    const int lastChromaPos   = ((OrigFracPos.x / iMvChromaFrac) * iMvChromaFrac) + FracMv.getHor() + (isIntChromaMv ? 0 : (chromaPixelAway << iMvShift)) ;

    const int lastPelPos    = std::max(lastLumaPos, lastChromaPos);
    // Verifica se a posição é considerada limpa com base no GDR
    if (lastPelPos < scaledEndX)
    {
      return true;    // posição limpa
    }
    else
    {
      return false;   // posição suja
    }
  }
  else
  {
    // refPic is normal picture     // Se a imagem de referência não está dentro de um intervalo GDR, verifica o GDR da imagem atual
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);
    // Retorna verdadeiro se a imagem atual está em um intervalo GDR, indicando que a posição é limpa
    if (isCurGdrPicture)
    {
      return false;     // posição suja
    }
    else
    {
      return true;      // posição limpa
    }
  }
}

/*
  A função isClean parece ser uma variação da função anterior, com um parâmetro adicional chamado ibc, que provavelmente indica se 
  o bloco é codificado usando o modo de predição "Intra Block Copy" (IBC).
*/
bool CodingStructure::isClean(const Position &IntPos, Mv FracMv, RefPicList e, int refIdx, bool ibc) const
{
  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */  // Verifica se o índice de referência é válido
  if (refIdx < 0) return false;

  Picture*   refPic;
  PicHeader *refPh;

  if (refIdx == MAX_NUM_REF)  // Obtém a imagem de referência correspondente
  {
    refPic = slice->getPic();
  }
  else
  {
    refPic = slice->getRefPic(e, refIdx);
  }

  if (!refPic)  // Verifica a validade da imagem de referência
  {
    return false;
  }

  if (refIdx == MAX_NUM_REF)  // Obtém o cabeçalho da imagem de referência
  {
    refPh = picHeader;
  }
  else
  {
    if (refPic->cs)
    {
      return false;
    }

    refPh = refPic->cs->picHeader;
  }

  if (!refPh)  // Verifica a validade do cabeçalho da imagem de referência
  {
    return false;
  }
  // Verifica se a imagem de referência está dentro de um intervalo GDR (Gradual Decoder Refresh)
  bool isRefGdrPicture = refPh->getInGdrInterval();

  if (isRefGdrPicture)  // Se a imagem de referência está dentro de um intervalo GDR
  {
    // Ajustes adicionais para verificar se a posição é limpa ou suja com base no GDR
    const int lumaPixelAway = 4;
    const int chromaPixelAway = 5;

    const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
    const int iMvLumaFrac = (1 << iMvShift);
    const int iMvChromaFrac = (iMvLumaFrac << 1);

    const bool isIntLumaMv = (FracMv.getHor() % iMvLumaFrac) == 0;
    const bool isIntChromaMv = (FracMv.getHor() % iMvChromaFrac) == 0;

    const int  scaledEndX = refPh->getVirtualBoundariesPosX(0) << iMvShift;

    const Position OrigFracPos = Position((IntPos.x) << iMvShift, IntPos.y << iMvShift);
    const int lastLumaPos = ((OrigFracPos.x / iMvLumaFrac)   * iMvLumaFrac) + FracMv.getHor() + (isIntLumaMv ? 0 : (lumaPixelAway << iMvShift));
    const int lastChromaPos = ((OrigFracPos.x / iMvChromaFrac) * iMvChromaFrac) + FracMv.getHor() + (isIntChromaMv ? 0 : (chromaPixelAway << iMvShift));

    const int lastPelPos = std::max(lastLumaPos, lastChromaPos);
    // Verifica se a posição é considerada limpa com base no GDR
    if (lastPelPos < scaledEndX)
    {
      return true;      //  posição limpa
    }
    else
    {
      return false;     // posição suja
    }
  }
  else
  {
    // refPic is normal picture    // Se a imagem de referência não está dentro de um intervalo GDR, verifica o GDR da imagem atual
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);
    // Retorna verdadeiro se a imagem atual está em um intervalo GDR, indicando que a posição é limpa
    if (isCurGdrPicture)
    {
      return false;     // posição suja
    }
    else
    {
      return true;      // posição limpa
    }
  }
}

/*
  A função isClean realiza uma verificação para determinar se uma determinada posição em uma imagem de referência é considerada 
  "limpa" ou "suja", com base nas configurações do Gradual Decoder Refresh (GDR). 
*/
bool CodingStructure::isClean(const Position &IntPos, RefPicList e, int refIdx) const
{
  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */  // Obtém a imagem de referência correspondente
  const Picture* const refPic = slice->getRefPic(e, refIdx);

  if (!refPic || refIdx < 0)  // Verifica a validade da imagem de referência e do índice de referência
  {
    return false;
  }

  PicHeader     *refPh = refPic->cs->picHeader;  // Obtém o cabeçalho da imagem de referência
  if (!refPh)  // Verifica a validade do cabeçalho da imagem de referência
  {
    return false;
  }
  // Verifica se a imagem de referência está dentro de um intervalo GDR (Gradual Decoder Refresh)
  bool isRefGdrPicture = refPh->getInGdrInterval();

  if (isRefGdrPicture)  // Se a imagem de referência está dentro de um intervalo GDR
  {
    if (IntPos.x < refPh->getVirtualBoundariesPosX(0))    // Verifica se a posição está dentro da área "limpa"
    {
      return true;    // posição limpa
    }
    else
    {
      return false;   // posição suja
    }
  }
  else
  {
    // refPic is normal picture    // Se a imagem de referência não está dentro de um intervalo GDR, verifica o GDR da imagem atual
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);

    if (isCurGdrPicture)    // Retorna verdadeiro se a imagem atual está em um intervalo GDR, indicando que a posição é limpa
    {
      return false;         // posção suja
    }
    else
    {
      return true;          // posição limpa
    }
  }
}

/*
  Essa função opera de maneira semelhante à anterior, mas em vez de receber uma referência específica da imagem de referência, ela recebe 
  diretamente um ponteiro para a Picture de referência. A lógica subjacente permanece a mesma, onde a função verifica se a posição 
  especificada é considerada "limpa" ou "suja" com base nas configurações do GDR.
*/
bool CodingStructure::isClean(const Position &IntPos, const Picture* const refPic) const
{
  if (!refPic)  // Verifica a validade da imagem de referência
  {
    return false;
  }

  PicHeader     *refPh = refPic->cs->picHeader;  // Obtém o cabeçalho da imagem de referência
  if (!refPh)  // Verifica a validade do cabeçalho da imagem de referência
  {
    return false;
  }
  // Verifica se a imagem de referência está dentro de um intervalo GDR (Gradual Decoder Refresh)
  bool isRefGdrPicture = refPh->getInGdrInterval();

  if (isRefGdrPicture)  // Se a imagem de referência está dentro de um intervalo GDR
  {
    // Verifica se a posição está dentro da área "limpa"
    if (IntPos.x < refPh->getVirtualBoundariesPosX(0))
    {
      return true;      // posição limpa
    }
    else
    {
      return false;     // posição suja
    }
  }
  else
  {
    // refPic is normal picture    // Se a imagem de referência não está dentro de um intervalo GDR, verifica o GDR da imagem atual
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);

    if (isCurGdrPicture)    // Retorna verdadeiro se a imagem atual está em um intervalo GDR, indicando que a posição é limpa
    {
      return false;         // posição suja
    }
    else
    {
      return true;          // posição limpa
    }
  }
}

/*
  Essa função isClean verifica se uma determinada posição (Intx, Inty) em um determinado tipo de canal (effChType) é considerada "limpa" 
  ou "suja" com base nas configurações do Gradual Decoder Refresh (GDR).
*/
bool CodingStructure::isClean(const int Intx, const int Inty, const ChannelType effChType) const
{
  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */  // Obtém o cabeçalho da imagem atual
  PicHeader     *curPh = picHeader;
  if (!curPh)  // Verifica a validade do cabeçalho da imagem atual
  {
    return false;
  }
  // Verifica se a imagem atual está dentro de um intervalo GDR (Gradual Decoder Refresh)
  bool isCurGdrPicture = curPh->getInGdrInterval();
  if (isCurGdrPicture)  // Se a imagem atual está dentro de um intervalo GDR
  {
    // Obtém a posição virtual de término da área limpa, ajustada pelo tipo de canal
    int virboundary_endx = curPh->getVirtualBoundariesPosX(0);

    virboundary_endx = virboundary_endx >> effChType;
    if (Intx < virboundary_endx)    // Verifica se a posição está dentro da área "limpa"
    {
      return true;        // posição limpa
    }
    else
    {
      return false;       // posição suja
    }
  }

  return true;           // Se a imagem atual não está dentro de um intervalo GDR, considera a posição como limpa
}

bool CodingStructure::isClean(const Position &IntPos, const ChannelType effChType) const
{
  bool ret = isClean(IntPos.x, IntPos.y, effChType);  // Chama a função isClean(int Intx, int Inty, ChannelType effChType)

  return ret;
}

bool CodingStructure::isClean(const Area &area, const ChannelType effChType) const
{
  // Obtém as posições dos cantos da área
  Position pTopLeft  = area.topLeft();
  Position pTopRight = area.topRight();
  Position pBotLeft  = area.bottomLeft();
  Position pBotRight = area.bottomRight();
  // Verifica se cada canto da área é considerado "limpo" com base no tipo de canal
  bool bTopLeft  = isClean(pTopLeft,  effChType);
  bool bTopRight = isClean(pTopRight, effChType);
  bool bBotLeft  = isClean(pBotLeft,  effChType);
  bool bBotRight = isClean(pBotRight, effChType);
  // Retorna true se todos os cantos da área forem "limpos"
  return bTopLeft && bTopRight && bBotLeft && bBotRight;
}

/*
  A função isClean(const ChannelType effChType) na classe CodingStructure verifica se a área Y (luminância) 
  é considerada "limpa" ou "suja" com base no tipo de canal (effChType)
*/
bool CodingStructure::isClean(const ChannelType effChType) const
{
  bool ret = isClean(area.Y(), effChType);  // Verifica se a área Y é considerada "limpa" com base no tipo de canal

  return ret;                               // retorna o resultado da verificação
}

/*
  A função isSubPuClean(PredictionUnit &pu, const Mv *mv) na classe CodingStructure verifica se cada subbloco de uma 
  unidade de predição (pu) é considerado "limpo" ou "sujo" com base nas informações de movimento e referência.
*/
bool CodingStructure::isSubPuClean(PredictionUnit &pu, const Mv *mv) const
{
  MotionBuf mb = pu.getMotionBuf();   // Obtém o buffer de movimento da unidade de predição

  if (pu.cu->affine)                  // Verifica se a unidade de predição é do tipo affine
  {
    Position puPos = pu.Y().pos();    // Obtém a posição e o tamanho do subbloco
    Size subPuSize = Size(4, 4);

    int isProf = 1;                   // alguma lógica específica do aplicativo (mim querer saber o que ser na real... hehe)
    
    // itera sobre os subblocos no buffer de movimento
    for (int y = 0; y < mb.height; y++)
    {
      for (int x = 0; x < mb.width; x++)
      {
        // Obtém as informações de movimento para o subbloco
        MotionInfo mi = mb.at(x, y);
        // Calcula a posição e a área do subbloco
        Position subPuPos  = Position{puPos.x + (x << 2), puPos.y + (y << 2)};
        Area     subPuArea = Area(subPuPos, subPuSize);
        Position subPuTR   = subPuArea.topRight();

        // check if SubPu with L0 is Out of boundary
        if (mi.refIdx[0] >= 0)
        {
          if (!isClean(subPuTR, mi.mv[0], REF_PIC_LIST_0, mi.refIdx[0], isProf))
          {
            return false;
          }
        }

        // check if SubPu wiht L1 is Out of boundary
        if (mi.refIdx[1] >= 0)
        {
          if (!isClean(subPuTR, mi.mv[1], REF_PIC_LIST_1, mi.refIdx[1], isProf))
          {
            return false;
          }
        }
      }
    }
  }
  // Retorna true se todos os subblocos forem considerados "limpos"
  return true;
}
#endif


/*
  Essa função serve para verificar se uma determinada posição está marcada como "decomposta" em uma estrutura de codificação. O conceito de 
  decomposição parece estar relacionado aos blocos e canais efetivos da imagem sendo processada.
*/
bool CodingStructure::isDecomp( const Position &pos, const ChannelType effChType )
{
  // Verifica se a posição está contida nos blocos do canal efetivo
  if( area.blocks[effChType].contains( pos ) )
  {
    // Retorna o valor correspondente no array m_isDecomp
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );        // Verifica no pai (se existir)
  }
  else
  {
    return false;                                     // Se não houver pai, retorna false
  }
}

// Verifica se a posição pos no canal efetivo effChType está marcada como "decomposta"
bool CodingStructure::isDecomp( const Position &pos, const ChannelType effChType ) const
{
  // Verifica se a posição está dentro dos blocos do canal efetivo
  if( area.blocks[effChType].contains( pos ) )
  {
    // Retorna o valor correspondente no array m_isDecomp
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  // Se a posição não está dentro dos blocos do canal efetivo
  else if( parent )
  {
    // Verifica no pai (se existir)
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;                                     // Se não há pai, retorna false
  }
}
//----------------------------------------------------------------
/*
  Essas funções têm como objetivo marcar áreas específicas como "decompostas" no contexto do objeto CodingStructure. A primeira função atua em uma área 
  específica de um componente, enquanto a segunda função itera sobre todos os blocos em uma área composta (UnitArea) e chama a primeira função para cada 
  bloco válido. As áreas marcadas como "decompostas" podem ser utilizadas em operações subsequentes durante o processo de codificação de vídeo.
*/
void CodingStructure::setDecomp(const CompArea &_area, const bool _isCoded /*= true*/)
{
  const UnitScale& scale = unitScale[_area.compID];  // Obtém a escala de unidade para o componente atual
  // Calcula a posição na matriz m_isDecomp usando a escala
  AreaBuf<bool> isCodedBlk( 
                            // Posição na matriz m_isDecomp
                            m_isDecomp[toChannelType( _area.compID )] + rsAddr( _area, area.blocks[_area.compID].pos(), area.blocks[_area.compID].width, scale ),
                            // Largura da área em blocos, ajustada pela escala no eixo x
                            area.blocks[_area.compID].width >> scale.posx,
                            // Largura da área em pixels, ajustada pela escala no eixo x
                            _area.width                     >> scale.posx,
                            // Altura da área em pixels, ajustada pela escala no eixo y
                            _area.height                    >> scale.posy);
  isCodedBlk.fill( _isCoded );  // Preenche a área com o valor _isCoded
}

void CodingStructure::setDecomp(const UnitArea &_area, const bool _isCoded /*= true*/)
{
  for( uint32_t i = 0; i < _area.blocks.size(); i++ )   // Itera sobre todos os blocos na UnitArea
  {
    if (_area.blocks[i].valid())                        // Verifica se o bloco é válido
    {
      setDecomp(_area.blocks[i], _isCoded);             // Chama a função setDecomp para o bloco atual
    }
  }
}
//----------------------------------------------------------------

/*
  A função signalModeCons parece estar relacionada à decisão de como o modo de divisão (split) deve ser sinalizado durante o processo de codificação.
*/
const int CodingStructure::signalModeCons( const PartSplit split, Partitioner &partitioner, const ModeType modeTypeParent ) const
{
  // Verifica condições para herança ou sinais diretos do modo de divisão
  if (CS::isDualITree(*this) || modeTypeParent != MODE_TYPE_ALL || partitioner.currArea().chromaFormat == CHROMA_444 || partitioner.currArea().chromaFormat == CHROMA_400 )
  {
    return LDT_MODE_TYPE_INHERIT;     // Se dual tree, modo de herança, ou formato de croma específico, sinaliza como herança
  }
  int minLumaArea = partitioner.currArea().lumaSize().area();
  // Ajusta a área mínima da luma com base no tipo de divisão
  if (split == CU_QUAD_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT) // the area is split into 3 or 4 parts
  {
    minLumaArea = minLumaArea >> 2;
  }
  else if (split == CU_VERT_SPLIT || split == CU_HORZ_SPLIT) // the area is split into 2 parts
  {
    minLumaArea = minLumaArea >> 1;
  }
  // Calcula o tamanho mínimo do bloco de croma com base no tamanho mínimo da área de luma
  int minChromaBlock = minLumaArea >> (getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, partitioner.currArea().chromaFormat) + getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, partitioner.currArea().chromaFormat));
  // Verifica se é um bloco croma 2xN especial
  bool is2xNChroma = (partitioner.currArea().chromaSize().width == 4 && split == CU_VERT_SPLIT) || (partitioner.currArea().chromaSize().width == 8 && split == CU_TRIV_SPLIT);
  // Decide o tipo de sinalização com base nas condições
  return minChromaBlock >= 16 && !is2xNChroma ? LDT_MODE_TYPE_INHERIT : ((minLumaArea < 32) || slice->isIntra()) ? LDT_MODE_TYPE_INFER : LDT_MODE_TYPE_SIGNAL;
}

/*
  A função clearCuPuTuIdxMap parece ser responsável por limpar e resetar índices associados a 
  CUs (Coding Units), PUs (Prediction Units) e TUs (Transform Units) em um mapa de índices.
*/
void CodingStructure::clearCuPuTuIdxMap( const UnitArea &_area, uint32_t numCu, uint32_t numPu, uint32_t numTu, uint32_t* pOffset )
{
  // Obtém a área da imagem após aplicar a área de recorte (_area) na imagem completa (*picture)
  UnitArea clippedArea = clipArea( _area, *picture );
  // Obtém o número de canais válidos para a área de croma especificada
  uint32_t numCh = ::getNumberValidChannels( _area.chromaFormat );
  // Itera sobre cada canal para limpar os índices associados a CUs, PUs e TUs
  for( uint32_t i = 0; i < numCh; i++ )
  {
    // Obtém as áreas da unidade para o canal atual
    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = clippedArea.blocks[i];
    // Obtém as escalas de unidade para o canal atual
    const UnitScale& scale = unitScale[_blk.compID];
    // Calcula as áreas escaladas para a unidade atual
    const Area scaledSelf = scale.scale( _selfBlk );
    const Area scaledBlk = scale.scale( _blk );
    // Calcula o deslocamento na matriz de índices
    const size_t offset = rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    // Limpa os índices associados a CUs, PUs e TUs para o canal atual
    unsigned *idxPtrCU = m_cuIdx[i] + offset;
    AreaBuf<uint32_t>( idxPtrCU, scaledSelf.width, scaledBlk.size() ).fill( 0 );

    unsigned *idxPtrPU = m_puIdx[i] + offset;
    AreaBuf<uint32_t>( idxPtrPU, scaledSelf.width, scaledBlk.size() ).fill( 0 );

    unsigned *idxPtrTU = m_tuIdx[i] + offset;
    AreaBuf<uint32_t>( idxPtrTU, scaledSelf.width, scaledBlk.size() ).fill( 0 );
  }

  //pop cu/pu/tus       // Remove CUs, PUs e TUs do final das listas, se necessário
  for( int i = m_numTUs; i > numTu; i-- )
  {
    m_tuCache.cache( tus.back() );
    tus.pop_back();
    m_numTUs--;
  }
  for( int i = m_numPUs; i > numPu; i-- )
  {
    m_puCache.cache( pus.back() );
    pus.pop_back();
    m_numPUs--;
  }
  for( int i = m_numCUs; i > numCu; i-- )
  {
    m_cuCache.cache( cus.back() );
    cus.pop_back();
    m_numCUs--;
  }
  for( int i = 0; i < 3; i++ )      // Atualiza os offsets
  {
    m_offsets[i] = pOffset[i];
  }
}

/*
  A função getLumaCU retorna um ponteiro para uma CodingUnit (CU) correspondente a uma posição específica na área de luma. 
*/
CodingUnit* CodingStructure::getLumaCU( const Position &pos )
{
  const ChannelType effChType = CHANNEL_TYPE_LUMA;          // Define o tipo de canal efetivo como LUMA
  const CompArea &_blk = area.blocks[effChType];            // Obtém a área do bloco para o canal efetivo
  CHECK( !_blk.contains( pos ), "must contain the pos" );  // Verifica se a posição está contida na área do bloco
  
  // Calcula o índice na matriz de índices de CUs para a posição dada
  const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

  if (idx != 0)         // Se o índice for diferente de zero, retorna o ponteiro para a CodingUnit correspondente
  {
    return cus[idx - 1];
  }
  else                  // Caso contrário, retorna um ponteiro nulo
  {
    return nullptr;
  }
}

/*
  O código getCU é usado para obter um ponteiro para um bloco de codificação (CodingUnit) em uma posição específica e uma componente de canal efetiva. 
  O código lida com casos em que a posição está fora da área ou quando a árvore é cromática e a componente é luma. Se isso acontecer, a função tenta 
  obter o bloco de codificação do pai (se existir). Caso contrário, se a posição está dentro da área, o código calcula o índice correspondente no array 
  de blocos de codificação (CUs) e retorna o bloco correspondente ou nullptr se o índice for zero.
*/
CodingUnit* CodingStructure::getCU( const Position &pos, const ChannelType effChType )
{
  const CompArea &_blk = area.blocks[effChType];  // Obtém a área efetiva da componente de canal especificada

  // Verifica se a posição está fora da área ou se a árvore é cromática e a componente é luma
  if( !_blk.contains( pos ) || (treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA) )
  {
    //keep this check, which is helpful to identify bugs
    if( treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA )
    {
      CHECK( parent == nullptr, "parent shall be valid; consider using function getLumaCU()" );
      CHECK( parent->treeType != TREE_D, "wrong parent treeType " );
    }
    if (parent)           // Se houver um pai, tenta obter o CU do pai para a posição
    {
      return parent->getCU(pos, effChType);
    }
    else
    {
      return nullptr;     // Não há pai, retorna nulo
    }
  }
  else
  {
    // Calcula o índice para a posição no array de CUs
    const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)         // Se o índice não for zero, retorna o CU correspondente
    {
      return cus[idx - 1];
    }
    else
    {
      return nullptr;     // Índice é zero, não há CU na posição
    }
  }
}

/*
  versão constante da função getCU. Ele mantém a mesma lógica, mas agora pode ser chamado em objetos const CodingStructure. 
  Isso significa que a função não pode modificar nenhum dos membros de dados do objeto CodingStructure.
*/
const CodingUnit* CodingStructure::getCU( const Position &pos, const ChannelType effChType ) const
{
  // Obtém a área efetiva da componente de canal especificada
  const CompArea &_blk = area.blocks[effChType];
  // Verifica se a posição está fora da área ou se a árvore é cromática e a componente é luma
  if( !_blk.contains( pos ) || (treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA) )
  { 
    // Mantém essa verificação, útil para identificar bugs
    if( treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA )
    {
      CHECK( parent == nullptr, "parent shall be valid; consider using function getLumaCU()" );
      CHECK( parent->treeType != TREE_D, "wrong parent treeType" );
    }
    if (parent)         // Se houver um pai, tenta obter o CU do pai para a posição
    {
      return parent->getCU(pos, effChType);
    }
    else
    {
      return nullptr;   // Não há pai, retorna nulo
    }
  }
  else
  {
    // Calcula o índice para a posição no array de CUs
    const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)           // Se o índice não for zero, retorna o CU correspondente
    {
      return cus[idx - 1];
    }
    else
    {
      return nullptr;       // Índice é zero, não há CU na posição
    }
  }
}

/*
  implementação da função getPU que obtém uma PredictionUnit para a posição especificada e a componente de canal efetiva. 
  Se a posição estiver fora da área, a função tenta obter a PU do pai, se existir. Se a posição estiver dentro da área, ela 
  calcula o índice para a posição no array de PUs e retorna a PU correspondente.
*/
PredictionUnit* CodingStructure::getPU( const Position &pos, const ChannelType effChType )
{
  const CompArea &_blk = area.blocks[effChType];  // Obtém a área efetiva da componente de canal especificada

  if( !_blk.contains( pos ) )                     // Verifica se a posição está fora da área
  {
    if (parent)                                  // Se houver um pai, tenta obter a PU do pai para a posição
    {
      return parent->getPU(pos, effChType);
    }
    else
    {
      return nullptr;                            // Não há pai, retorna nulo
    }
  }
  else
  {
    // Calcula o índice para a posição no array de PUs
    const unsigned idx = m_puIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)                                // Se o índice não for zero, retorna a PU correspondente
    {
      return pus[idx - 1];
    }
    else
    {
      return nullptr;                            // Índice é zero, não há PU na posição
    }
  }
}

// Versão constante da função anterior (getPU), ou seja, não altera o estado do objeto, sendo chamada em objs 'const' do tipo 'CodingStructure'
const PredictionUnit * CodingStructure::getPU( const Position &pos, const ChannelType effChType ) const
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getPU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_puIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)
    {
      return pus[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

// Lógica semelhante às funções getCU e getPU. Retorna um ponteiro para a TransformUnit (TU) na posição especificada, para o canal de tipo efetivo ('effChType')...
TransformUnit* CodingStructure::getTU( const Position &pos, const ChannelType effChType, const int subTuIdx )
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];

        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while( !tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].contains( pos ) )
            {
              extraIdx++;
              CHECK( tus[idx - 1 + extraIdx]->cu->treeType == TREE_C, "tu searched by position points to a chroma tree CU" );
              CHECK( extraIdx > 3, "extraIdx > 3" );
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
    else if (m_isTuEnc)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
}

// Versão constante da função 'getTU'
const TransformUnit * CodingStructure::getTU( const Position &pos, const ChannelType effChType, const int subTuIdx ) const
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];
        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while ( !tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].contains(pos) )
            {
              extraIdx++;
              CHECK( tus[idx - 1 + extraIdx]->cu->treeType == TREE_C, "tu searched by position points to a chroma tree CU" );
              CHECK( extraIdx > 3, "extraIdx > 3" );
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
    else if (m_isTuEnc)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
}

//  Essa função é usada para adicionar uma nova CU à estrutura de codificação, inicializando seus parâmetros e atualizando os índices necessários.
CodingUnit& CodingStructure::addCU( const UnitArea &unit, const ChannelType chType )
{
  CodingUnit *cu = m_cuCache.get();   // Obtém uma instância de CodingUnit do cache

  cu->UnitArea::operator=( unit );    // Copia a área da unidade fornecida para a CU
  cu->initData();                     // Inicializa os dados da CU
  // Configura os ponteiros e propriedades da CU
  cu->cs        = this;
  cu->slice     = nullptr;
  cu->next      = nullptr;
  cu->firstPU   = nullptr;
  cu->lastPU    = nullptr;
  cu->firstTU   = nullptr;
  cu->lastTU    = nullptr;
  cu->chType    = chType;
  cu->treeType = treeType;
  cu->modeType = modeType;
  
  // Adiciona a CU à lista de CUs
  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if( prevCU )
  {
    prevCU->next = cu;
  }

  cus.push_back( cu );

  uint32_t idx = ++m_numCUs;
  cu->idx  = idx;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );

  // Atualiza os índices da CU na matriz m_cuIdx para cada canal
  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = cu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_cuIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    // Verifica se o índice já está ocupado; deve ser '0'
    CHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
    // Preenche a área correspondente com o índice da CU
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }

  return *cu;  // Retorna a referência para a CU recém-adicionada
}

// Responsável por adicionar uma nova unidade de predição (PU) à estrutura de codificação.
PredictionUnit& CodingStructure::addPU( const UnitArea &unit, const ChannelType chType )
{
  PredictionUnit *pu = m_puCache.get();   // Obtém uma instância de PredictionUnit do cache

  pu->UnitArea::operator=( unit );        // Copia a área da unidade fornecida para a PU
  pu->initData();                         // Inicializa os dados da PU
  // Configura os ponteiros e propriedades da PU
  pu->next   = nullptr;
  pu->cs     = this;
  pu->cu     = m_isTuEnc ? cus[0] : getCU( unit.blocks[chType].pos(), chType );
  pu->chType = chType;

  // Adiciona a PU à lista de PUs
  PredictionUnit *prevPU = m_numPUs > 0 ? pus.back() : nullptr;

  if( prevPU && prevPU->cu == pu->cu )
  {
    prevPU->next = pu;
  }

  pus.push_back( pu );

  // Atualiza os ponteiros firstPU e lastPU na CU associada à PU
  if( pu->cu->firstPU == nullptr )
  {
    pu->cu->firstPU = pu;
  }
  pu->cu->lastPU = pu;

  uint32_t idx = ++m_numPUs;
  pu->idx  = idx;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );
  // Atualiza os índices da PU na matriz m_puIdx para cada canal
  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !pu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = pu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_puIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    // Verifica se o índice já está ocupado; deve ser '0'
    CHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
    // Preenche a área correspondente com o índice da PU
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }

  return *pu;  // Retorna a referência para a PU recém-adicionada
}

// Responsável por adicionar uma nova Unidade de Transformação (TU) à estrutura de codificação. 
TransformUnit& CodingStructure::addTU( const UnitArea &unit, const ChannelType chType )
{
  TransformUnit *tu = m_tuCache.get();    // Obtém uma instância de TransformUnit do cache

  tu->UnitArea::operator=( unit );        // Copia a área da unidade fornecida para a TU
  tu->initData();                         // Inicializa os dados da TU
  // Configura os ponteiros e propriedades da TU
  tu->next   = nullptr;
  tu->prev   = nullptr;
  tu->cs     = this;
  tu->cu     = m_isTuEnc ? cus[0] : getCU( unit.blocks[chType].pos(), chType );
  tu->chType = chType;

  // Obtém a TU anterior, se houver, e ajusta os ponteiros
  TransformUnit *prevTU = m_numTUs > 0 ? tus.back() : nullptr;

  if( prevTU && prevTU->cu == tu->cu )
  {
    prevTU->next = tu;
    tu->prev     = prevTU;
  }

  tus.push_back( tu );    // Adiciona a TU à lista de TUs

  // Atualiza os ponteiros firstTU e lastTU na CU correspondente
  if( tu->cu )
  {
    if( tu->cu->firstTU == nullptr )
    {
      tu->cu->firstTU = tu;
    }
    tu->cu->lastTU = tu;
  }

  // Atualiza o índice e as estruturas de dados para cada canal
  uint32_t idx = ++m_numTUs;
  tu->idx  = idx;

  // Configura os ponteiros para os coeficientes, dados PCM e tipos de execução para cada canal
  TCoeff *coeffs[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
  Pel    *pcmbuf[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
  bool   *runType[5]   = { nullptr, nullptr, nullptr, nullptr, nullptr };

  uint32_t numCh = ::getNumberValidComponents( area.chromaFormat );

  // Para cada canal válido
  for (uint32_t i = 0; i < numCh; i++)
  {
    if (!tu->blocks[i].valid())    // Se o bloco atual não for válido, continua para o próximo canal
    {
      continue;
    }
    // Se for um canal cromático
    if (i < ::getNumberValidChannels(area.chromaFormat))
    {
      const CompArea &_selfBlk = area.blocks[i];
      const CompArea     &_blk = tu->blocks[i];

      bool isIspTu = tu->cu != nullptr && tu->cu->ispMode && isLuma(_blk.compID);

      bool isFirstIspTu = false;
      if (isIspTu)
      {
        isFirstIspTu = CU::isISPFirst(*tu->cu, _blk, getFirstComponentOfChannel(ChannelType(i)));
      }
      // Se não for uma TU ISP ou for a primeira TU ISP
      if (!isIspTu || isFirstIspTu)
      {
        const UnitScale& scale = unitScale[_blk.compID];

        const Area scaledSelf = scale.scale(_selfBlk);
        const Area scaledBlk = isIspTu ? scale.scale(tu->cu->blocks[i]) : scale.scale(_blk);
        unsigned *idxPtr = m_tuIdx[i] + rsAddr(scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width);
        // Verifica se o índice já está ocupado; deve ser '0'
        CHECK(*idxPtr, "Overwriting a pre-existing value, should be '0'!");
        // Preenche a área correspondente com o índice da TU
        AreaBuf<uint32_t>(idxPtr, scaledSelf.width, scaledBlk.size()).fill(idx);
      }
    }
    // Configura os ponteiros para os coeficientes e dados PCM
    coeffs[i] = m_coeffs[i] + m_offsets[i];
    pcmbuf[i] = m_pcmbuf[i] + m_offsets[i];

    // Configura os ponteiros para o tipo de execução, se disponível
    if (i < MAX_NUM_CHANNEL_TYPE)
    {
      if (m_runType[i] != nullptr)
      {
        runType[i] = m_runType[i] + m_offsets[i];
      }
    }

    // Atualiza o deslocamento para o próximo bloco
    unsigned areaSize = tu->blocks[i].area();
    m_offsets[i] += areaSize;
  }
  tu->init(coeffs, pcmbuf, runType);    // Inicializa a TU com os ponteiros configurados

  return *tu;                           // Retorna a referência para a TU recém-ad
}

/*----------------------------------------------------------------
  Essa função percorre a estrutura de codificação e adiciona TUs vazias às áreas correspondentes com a profundidade da transformação adequada. Se a área 
  atual puder ser dividida, a função é chamada recursivamente para cada subárea; caso contrário, uma única TU é adicionada com coeficientes e buffer PCM 
  inicializados com zeros.
*/
void CodingStructure::addEmptyTUs( Partitioner &partitioner )
{
  const UnitArea& area    = partitioner.currArea();                             // Obtém a área atual do particionador
  bool            split   = partitioner.canSplit(TU_MAX_TR_SPLIT, *this);       // Verifica se é possível dividir a área atual em TUs de tamanho máximo
  const unsigned  trDepth = partitioner.currTrDepth;                            // Obtém a profundidade da transformação atual

  if( split )                                                 // Se a área pode ser dividida
  {
    partitioner.splitCurrArea( TU_MAX_TR_SPLIT, *this );      // Divide a área atual em TUs de tamanho máximo
    do                                                        // Recursivamente chama a função para cada subárea
    {
      addEmptyTUs( partitioner );
    } while( partitioner.nextPart( *this ) );

    partitioner.exitCurrSplit();                              // Sai da divisão atual
  }
  else
  {
    // Adiciona uma nova TU à estrutura de codificação com a área atual e o tipo de canal do particionador
    TransformUnit &tu = this->addTU( CS::getArea( *this, area, partitioner.chType ), partitioner.chType );
    // Obtém o número de blocos válidos para a transformação
    unsigned numBlocks = ::getNumberValidTBlocks( *this->pcv );
    // Para cada componente (por exemplo, Y, Cb, Cr)
    for( unsigned compID = COMPONENT_Y; compID < numBlocks; compID++ )
    {
      // Se o bloco da TU para a componente atual for válido
      if( tu.blocks[compID].valid() )
      {
        // Preenche os coeficientes e o buffer PCM com zeros
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
        tu.getPcmbuf( ComponentID( compID ) ).fill( 0 );
      }
    }
    tu.depth = trDepth;    // Ajusta a profundidade da TU
  }
}

/// @brief A função traverseCUs é responsável por criar e retornar um objeto CUTraverser, que é utilizado para percorrer através de CodingUnits (CUs) 
/// dentro de uma UnitArea especificada e do tipo de canal efetivo (effChType). Essa função é útil para iterar sobre CUs em uma área específica da 
/// estrutura de codificação (CodingStructure).
/// @param unit 
/// @param effChType 
/// @return CUTraverser( firstCU, lastCU )
CUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType )
{
  // Obtém o primeiro CU na posição correspondente à posição luma ou chroma
  CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  CodingUnit* lastCU = firstCU;
  if( !CS::isDualITree( *this ) )   //for a more generalized separate tree  // Se a estrutura de codificação não for uma árvore dupla
  {
    bool bContinue = true;
    CodingUnit* currCU = firstCU;
    while( bContinue )              // Loop para encontrar o último CU dentro da área especificada
    {
      if( currCU == nullptr )      // Se o CU atual for nulo, indica o final da lista de CUs
      {
        bContinue = false;
        lastCU = currCU;
      }
      else if( currCU->chType != effChType )
      {
        // Se o tipo de canal do CU não corresponder ao tipo efetivo, avança para o próximo CU
        lastCU = currCU;
        currCU = currCU->next;
      }
      else
      {
        if( unit.contains( *currCU ) )    // Se o CU atual estiver dentro da área especificada
        {
          lastCU = currCU;
          currCU = currCU->next;
        }
        else
        {
          bContinue = false;          // Se o CU atual não estiver dentro da área, interrompe o loop
          lastCU = currCU;
        }
      }
    }
  }
  else
  {
    do    // Loop para encontrar o último CU dentro da área especificada em uma árvore dupla
    {
    } while (lastCU && (lastCU = lastCU->next) && unit.contains(*lastCU));
  }
  // Retorna um objeto CUTraverser com os CUs inicial e final
  return CUTraverser( firstCU, lastCU );
}

PUTraverser CodingStructure::traversePUs( const UnitArea& unit, const ChannelType effChType )
{
  // Obtém o primeiro PU na posição correspondente à posição luma ou chroma
  PredictionUnit* firstPU = getPU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  PredictionUnit* lastPU  = firstPU;

  // Loop para encontrar o último PU dentro da área especificada
  do { } while( lastPU && ( lastPU = lastPU->next ) && unit.contains( *lastPU ) );
  // Retorna um objeto PUTraverser com os PUs inicial e final
  return PUTraverser( firstPU, lastPU );
}

TUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType )
{
  // Obtém o primeiro TU na posição correspondente à posição luma ou chroma
  TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  TransformUnit* lastTU  = firstTU;
  // Loop para encontrar o último TU dentro da área especificada
  do { } while( lastTU && ( lastTU = lastTU->next ) && unit.contains( *lastTU ) );
  // Retorna um objeto TUTraverser com os TUs inicial e final
  return TUTraverser( firstTU, lastTU );
}

// Versão const de tranverseCUs
cCUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const CodingUnit* lastCU  = firstCU;

  do { } while( lastCU && ( lastCU = lastCU->next ) && unit.contains( *lastCU ) );

  return cCUTraverser( firstCU, lastCU );
}

// Versão const de transversePUs
cPUTraverser CodingStructure::traversePUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const PredictionUnit* firstPU = getPU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const PredictionUnit* lastPU  = firstPU;

  do { } while( lastPU && ( lastPU = lastPU->next ) && unit.contains( *lastPU ) );

  return cPUTraverser( firstPU, lastPU );
}

// Versão const de transverseTUs
cTUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && ( lastTU = lastTU->next ) && unit.contains( *lastTU ) );

  return cTUTraverser( firstTU, lastTU );
}

// coding utilities

// Função para alocar espaço para vetores (cus, pus, tus) a nível de imagem (pic level).
void CodingStructure::allocateVectorsAtPicLevel()
{
  // Verifica se é necessário alocar o dobro de espaço para CUs, PUs e TUs
  const int  twice = ( !pcv->ISingleTree && slice->isIRAP() && pcv->chrFormat != CHROMA_400 ) ? 2 : 1;
  // Tamanho da alocação, considerando a área do bloco de maior granularidade.
  size_t allocSize = twice * unitScale[0].scale( area.blocks[0].size() ).area();

  // Reserva espaço nos vetores para CUs, PUs e TUs
  cus.reserve( allocSize );
  pus.reserve( allocSize );
  tus.reserve( allocSize );
}

// Criação da estrutura de codificação para uma área e formato cromático específicos.
#if GDR_ENABLED
void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused, const bool isGdrEnabled)
#else
void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused)
#endif
{
    // Inicialização da estrutura de codificação com base na área e formato cromático.
#if GDR_ENABLED
  m_gdrEnabled = isGdrEnabled;
#endif

  // Cria as estruturas internas com base na área e configurações fornecidas
  createInternals(UnitArea(_chromaFormat, _area), isTopLayer, isPLTused);

  // Se for a camada superior, retorna sem continuar
  if (isTopLayer)
  {
    return;
  }

#if GDR_ENABLED                   // Se GDR estiver ativado, inicializa o cabeçalho da imagem (PicHeader)
  if (m_gdrEnabled)
  {
    picHeader = new PicHeader();
    picHeader->initPicHeader();
  }
#endif

  // Cria as estruturas de reconstrução, predição, resíduo e original com base na área
  m_reco.create( area );
  m_pred.create( area );
  m_resi.create( area );
  m_orgr.create( area );
}

// Sobrecarga da função create para uma unidade específica.
#if GDR_ENABLED
void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused, const bool isGdrEnabled)
#else
void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused)
#endif
{
    // Inicialização da estrutura de codificação com base na unidade, indicando se é a camada superior e se PLT é utilizada.
#if GDR_ENABLED
  m_gdrEnabled = isGdrEnabled;
#endif

  // Cria as estruturas internas com base na unidade e configurações fornecidas
  createInternals(_unit, isTopLayer, isPLTused);

  if (isTopLayer)  // Se for a camada superior, retorna sem continuar
  {
    return;
  }

#if GDR_ENABLED                   // Se GDR estiver ativado, inicializa o cabeçalho da imagem (PicHeader)
  if (m_gdrEnabled)
  {
    picHeader = new PicHeader();
    picHeader->initPicHeader();
  }
#endif

  // Cria as estruturas de reconstrução, predição, resíduo e original com base na unidade
  m_reco.create( area );
  m_pred.create( area );
  m_resi.create( area );
  m_orgr.create( area );
}

/// @brief função createInternals é responsável por inicializar os membros de dados internos da estrutura de codificação CodingStructure.
/// @param _unit 
/// @param isTopLayer 
/// @param isPLTused 
void CodingStructure::createInternals(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused)
{
  area = _unit;  // Atribui a área passada como argumento à variável 'area'

  // Copia a escala de unidade correspondente à crominância do UnitScaleArray para o unitScale da CodingStructure
  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;
  parent  = nullptr;

  // Aloca e inicializa arrays para índices de CUs, PUs, TUs e um array de bool para marcação de decomposição, para cada canal/chroma válido.
  unsigned numCh = ::getNumberValidChannels(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    unsigned _area = unitScale[i].scale( area.blocks[i].size() ).area();

    m_cuIdx[i]    = _area > 0 ? new unsigned[_area] : nullptr;
    m_puIdx[i]    = _area > 0 ? new unsigned[_area] : nullptr;
    m_tuIdx[i]    = _area > 0 ? new unsigned[_area] : nullptr;
    m_isDecomp[i] = _area > 0 ? new bool    [_area] : nullptr;
  }
  // Inicializa offsets para cada componente/chroma
  numCh = getNumberValidComponents(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    m_offsets[i] = 0;
  }
  // Se não for a camada superior, cria e inicializa coeficientes na camada de codificação
  if (!isTopLayer)
  {
    createCoeffs(isPLTused);
  }
  // Calcula a área escalada para a crominância, considerando o dimensionamento de MI
  unsigned _lumaAreaScaled = g_miScaling.scale( area.lumaSize() ).area();
  // Aloca espaço para armazenar informações de movimento (MotionInfo) para a área de luma
  m_motionBuf       = new MotionInfo[_lumaAreaScaled];
  // Inicializa dados estruturais adicionais
  initStructData();
}

/// @brief Adiciona um elemento MotionInfo à tabela de candidatos (lut) mantendo no máximo MAX_NUM_HMVP_CANDS candidatos únicos.
/// @param lut  (provavelmente, uma tabela de hash de candidatos para Motion Vector Prediction)
/// @param mi (motion info)
void CodingStructure::addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> &lut, const MotionInfo &mi)
{
  size_t currCnt = lut.size();
  // Verifica se o candidato já existe na tabela
  bool pruned      = false;
  int  sameCandIdx = 0;

  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }
  
  // Se o candidato já existe ou a tabela está cheia, remove o candidato duplicado ou mais antigo
  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);      // Adiciona o novo candidato à tabela
}

/// @brief // Reinicia as informações sobre paletas de cores anteriores (prevPLT), definindo os tamanhos de paletas como zero 
/// e limpando os dados das paletas.
/// @param prevPLT 
void CodingStructure::resetPrevPLT(PLTBuf& prevPLT)
{
  // Zera os tamanhos das paletas para cada componente/canal.
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    prevPLT.curPLTSize[comp] = 0;
  }

  // Preenche com zeros os dados das paletas para cada componente.
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    memset(prevPLT.curPLT[comp], 0, MAXPLTPREDSIZE * sizeof(Pel));
  }
}

/// @brief Reorganiza as informações sobre paletas de cores anteriores (prevPLT) para considerar novas paletas e reutilizar paletas existentes, 
/// ajustando os tamanhos e os dados das paletas.
/// @param prevPLT 
/// @param curPLTSize 
/// @param curPLT 
/// @param reuseflag 
/// @param compBegin 
/// @param numComp 
/// @param jointPLT 
void CodingStructure::reorderPrevPLT(PLTBuf& prevPLT, uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE], Pel curPLT[MAX_NUM_COMPONENT][MAXPLTSIZE], bool reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE], uint32_t compBegin, uint32_t numComp, bool jointPLT)
{
  // Arrays temporários para armazenar dados de paletas reorganizadas.
  Pel stuffedPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE];
  uint8_t tempCurPLTsize[MAX_NUM_CHANNEL_TYPE];
  uint8_t stuffPLTsize[MAX_NUM_COMPONENT];

  // Tamanhos máximos permitidos para predição de paletas, dependendo se é uma predição conjunta ou separada.
  uint32_t maxPredPltSize = jointPLT ? MAXPLTPREDSIZE : MAXPLTPREDSIZE_DUALTREE;

  // Inicialização dos arrays temporários com dados das paletas atuais.
  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    tempCurPLTsize[comID] = curPLTSize[comID];
    stuffPLTsize[i] = 0;
    memcpy(stuffedPLT[i], curPLT[i], curPLTSize[comID] * sizeof(Pel));
  }

  // Iteração sobre os canais para reorganizar paletas existentes e reutilizar paletas quando possível.
  for (int ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((ch > 0) ? COMPONENT_Cb : COMPONENT_Y);
    if (ch > 1) break;
    // Iteração sobre paletas existentes para verificar reutilização.
    for (int i = 0; i < prevPLT.curPLTSize[comID]; i++)
    {
      // Verifica se há espaço para reutilizar esta paleta.
      if (tempCurPLTsize[comID] + stuffPLTsize[ch] >= maxPredPltSize)
      {
        break;
      }

      // Verifica se a paleta não está marcada para reutilização.
      if (!reuseflag[comID][i])
      {
        // Adiciona a paleta ao array temporário de paletas reorganizadas.
        if (ch == COMPONENT_Y)
        {
          stuffedPLT[0][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[0][i];
        }
        else
        {
          stuffedPLT[1][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[1][i];
          stuffedPLT[2][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[2][i];
        }
        stuffPLTsize[ch]++;
      }
    }
  }

  // Atualiza os tamanhos e os dados das paletas de cores anteriores.
  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    prevPLT.curPLTSize[comID] = curPLTSize[comID] + stuffPLTsize[comID];
    memcpy(prevPLT.curPLT[i], stuffedPLT[i], prevPLT.curPLTSize[comID] * sizeof(Pel));
    CHECK(prevPLT.curPLTSize[comID] > maxPredPltSize, " Maximum palette predictor size exceed limit");
  }
}

/// @brief Configura as informações de paletas de cores anteriores (prevPLT) com base em um objeto PLTBuf fornecido (predictor).
/// @param predictor 
void CodingStructure::setPrevPLT(PLTBuf predictor)
{
  // Copia os tamanhos de paletas para cada componente/canal.
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    prevPLT.curPLTSize[comp] = predictor.curPLTSize[comp];
  }
  // Copia os dados das paletas para cada componente.
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    memcpy(prevPLT.curPLT[comp], predictor.curPLT[comp], MAXPLTPREDSIZE * sizeof(Pel));
  }
}

/// @brief ealiza a cópia das informações de tamanhos e dados de paletas do objeto prevPLT associado à estrutura de codificação 
/// (CodingStructure) para um objeto PLTBuf chamado predictor.
/// @param predictor 
// Armazena as informações de paletas de cores anteriores (prevPLT) em um objeto PLTBuf fornecido (predictor).
void CodingStructure::storePrevPLT(PLTBuf& predictor)
{
  // Copia os tamanhos de paletas para cada componente/canal.
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    predictor.curPLTSize[comp] = prevPLT.curPLTSize[comp];
  }
  // Copia os dados das paletas para cada componente.
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    memcpy(predictor.curPLT[comp], prevPLT.curPLT[comp], MAXPLTPREDSIZE * sizeof(Pel));
  }
}

/// @brief Reassocia os buffers de imagem (picture) com os buffers internos de reconstrução (m_reco),
// de predição (m_pred), e de resíduo (m_resi) na estrutura de codificação.
// Esta função é projetada para ser utilizada apenas no nível superior da estrutura de codificação.
void CodingStructure::rebindPicBufs()
{
  // Verifica se a função está sendo chamada para uma estrutura de codificação no nível superior.
  CHECK( parent, "rebindPicBufs can only be used for the top level CodingStructure" );

  // Associa os buffers de reconstrução (m_reco) com os buffers de imagem de reconstrução.
  if (!picture->M_BUFS(0, PIC_RECONSTRUCTION).bufs.empty())
  {
    m_reco.createFromBuf(picture->M_BUFS(0, PIC_RECONSTRUCTION));
  }
  else
  {
    m_reco.destroy();
  }
  // Associa os buffers de predição (m_pred) com os buffers de imagem de predição.
  if (!picture->M_BUFS(0, PIC_PREDICTION).bufs.empty())
  {
    m_pred.createFromBuf(picture->M_BUFS(0, PIC_PREDICTION));
  }
  else
  {
    m_pred.destroy();
  }
  // Associa os buffers de resíduo (m_resi) com os buffers de imagem de resíduo.
  if (!picture->M_BUFS(0, PIC_RESIDUAL).bufs.empty())
  {
    m_resi.createFromBuf(picture->M_BUFS(0, PIC_RESIDUAL));
  }
  else
  {
    m_resi.destroy();
  }
  // Se a estrutura de codificação for um codificador, associa os buffers de organização de resíduo (m_orgr).
  if( pcv->isEncoder )
  {
    if (!picture->M_BUFS(0, PIC_RESIDUAL).bufs.empty())
    {
      m_orgr.create(area.chromaFormat, area.blocks[0], pcv->maxCUWidth);
    }
    else
    {
      m_orgr.destroy();
    }
  }
}

/// @brief Cria os arrays de coeficientes e de buffer de pixels (PCMs) para cada componente válida da estrutura de codificação.
// Se a utilização de Paleta é ativada (isPLTused), também cria os arrays de tipo de execução de codificação (runType) para cada componente de crominância.
/// @param isPLTused 
void CodingStructure::createCoeffs(const bool isPLTused)
{
  // Obtém o número de componentes válidas para a crominância.
  const unsigned numCh = getNumberValidComponents( area.chromaFormat );

  // Para cada componente válida, aloca os arrays de coeficientes e buffer de pixels (PCMs).
  for( unsigned i = 0; i < numCh; i++ )
  {
    unsigned _area = area.blocks[i].area();

    // Aloca o array de coeficientes para a área da componente atual.
    m_coeffs[i] = _area > 0 ? ( TCoeff* ) xMalloc( TCoeff, _area ) : nullptr;
    // Aloca o array de buffer de pixels (PCMs) para a área da componente atual.
    m_pcmbuf[i] = _area > 0 ? ( Pel*    ) xMalloc( Pel,    _area ) : nullptr;
  }

  if (isPLTused)  // Se a utilização de Paleta é ativada, aloca os arrays de tipo de execução de codificação (runType) para cada componente de crominância.
  {
    // Define a quantidade de componentes de crominância, considerando se a crominância está ativada.
    // Para cada componente de crominância válida, aloca o array de tipo de execução de codificação (runType).
    for (unsigned i = 0; i < (isChromaEnabled(area.chromaFormat) ? 2 : 1); i++)
    {
      unsigned _area = area.blocks[i].area();
      // Aloca o array de tipo de execução de codificação (runType) para a área da componente de crominância atual.
      m_runType[i] = _area > 0 ? (bool*)xMalloc(bool, _area) : nullptr;
    }
  }
}

/// @brief // Desaloca a memória alocada para os coeficientes, buffers de pixels (PCMs) e tipos de execução de codificação (runType).
void CodingStructure::destroyCoeffs()
{
  // Para cada componente de codificação, libera a memória alocada para os coeficientes e buffers de pixels (PCMs).
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if (m_coeffs[i])    // Verifica se o array de coeficientes foi alocado e, se sim, libera a memória e define o ponteiro como nulo.
    {
      xFree(m_coeffs[i]);
      m_coeffs[i] = nullptr;
    }
    if (m_pcmbuf[i])    // Verifica se o array de buffer de pixels (PCMs) foi alocado e, se sim, libera a memória e define o ponteiro como nulo.
    {
      xFree(m_pcmbuf[i]);
      m_pcmbuf[i] = nullptr;
    }
  }

  // Para cada tipo de execução de codificação (runType) de crominância, libera a memória alocada.
  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    if (m_runType[i])    // Verifica se o array de tipos de execução de codificação (runType) foi alocado e, se sim, libera a memória e define o ponteiro como nulo.
    {
      xFree(m_runType[i]);
      m_runType[i] = nullptr;
    }
  }
}

/// @brief Inicializa a subestrutura (subStruct) com base em uma área específica (subArea) e outras configurações.
/// @param subStruct 
/// @param _chType 
/// @param subArea 
/// @param isTuEnc 
void CodingStructure::initSubStructure( CodingStructure& subStruct, const ChannelType _chType, const UnitArea &subArea, const bool &isTuEnc )
{
  // Verifica se está tentando inicializar a própria estrutura como subestrutura, o que não é permitido.
  CHECK( this == &subStruct, "Trying to init self as sub-structure" );

  // Configurações iniciais para a subestrutura.
  subStruct.useDbCost = false;
  subStruct.costDbOffset = 0;

  // Atualiza as posições da subestrutura com base na área fornecida.
  for( uint32_t i = 0; i < subStruct.area.blocks.size(); i++ )
  {
    CHECKD( subStruct.area.blocks[i].size() != subArea.blocks[i].size(), "Trying to init sub-structure of incompatible size" );

    subStruct.area.blocks[i].pos() = subArea.blocks[i].pos();
  }

  // Verifica se a subestrutura está contida na estrutura pai (se existir).
  if( parent )
  {
    // allow this to be false at the top level (need for edge CTU's)
    CHECKD( !area.contains( subStruct.area ), "Trying to init sub-structure not contained in the parent" );
  }

  // Configurações de contexto compartilhadas entre a estrutura pai e a subestrutura.
  subStruct.parent    = this;
  subStruct.picture   = picture;

  subStruct.sps       = sps;
  subStruct.vps       = vps;
  subStruct.pps       = pps;
#if GDR_ENABLED
  if (m_gdrEnabled)
  {
    if (!subStruct.picHeader)
    {
      subStruct.picHeader = new PicHeader;
      subStruct.picHeader->initPicHeader();
    }
    *subStruct.picHeader = *picHeader;
  }
  else
  {
    subStruct.picHeader = picHeader;
  }
#else
  subStruct.picHeader = picHeader;
#endif

  memcpy(subStruct.alfApss, alfApss, sizeof(alfApss));

  subStruct.lmcsAps = lmcsAps;
  subStruct.scalinglistAps = scalinglistAps;

  subStruct.slice     = slice;
  subStruct.baseQP    = baseQP;
  subStruct.prevQP[_chType]
                      = prevQP[_chType];
  subStruct.pcv       = pcv;

  subStruct.m_isTuEnc = isTuEnc;

  subStruct.motionLut = motionLut;

  subStruct.prevPLT = prevPLT;

  subStruct.treeType  = treeType;
  subStruct.modeType  = modeType;

  subStruct.initStructData( currQP[_chType] );

  if( isTuEnc )  // Se for uma subestrutura para codificação de TU, copia as CUs e PUs correspondentes.
  {
    CHECKD( area != subStruct.area, "Trying to init sub-structure for TU-encoding of incompatible size" );

    for( const auto &pcu : cus )
    {
      CodingUnit &cu = subStruct.addCU( *pcu, _chType );

      cu = *pcu;
    }

    for( const auto &ppu : pus )
    {
      PredictionUnit &pu = subStruct.addPU( *ppu, _chType );

      pu = *ppu;
    }
    // Copia as marcações de decomposição da estrutura pai para a subestrutura.
    unsigned numComp = ::getNumberValidChannels( area.chromaFormat );
    for( unsigned i = 0; i < numComp; i++)
    {
      ::memcpy( subStruct.m_isDecomp[i], m_isDecomp[i], (unitScale[i].scale( area.blocks[i].size() ).area() * sizeof( bool ) ) );
    }
  }
}

void CodingStructure::useSubStructure( const CodingStructure& subStruct, const ChannelType chType, const UnitArea &subArea, const bool cpyPred /*= true*/, const bool cpyReco /*= true*/, const bool cpyOrgResi /*= true*/, const bool cpyResi /*= true*/, const bool updateCost /*= true*/ )
{
  // Recorta a área da subestrutura para garantir que esteja dentro dos limites da imagem.
  UnitArea clippedArea = clipArea( subArea, *picture );
  // Define a decompensação para a área recortada.
  setDecomp( clippedArea );
  // Cria buffers para cópia de predição, resíduo e reconstrução, conforme especificado pelos parâmetros.
  CPelUnitBuf subPredBuf = cpyPred ? subStruct.getPredBuf( clippedArea ) : CPelUnitBuf();
  CPelUnitBuf subResiBuf = cpyResi ? subStruct.getResiBuf( clippedArea ) : CPelUnitBuf();
  CPelUnitBuf subRecoBuf = cpyReco ? subStruct.getRecoBuf( clippedArea ) : CPelUnitBuf();

  if( parent )  // Se a instância atual tiver um pai, copia os buffers para os buffers correspondentes no picture associado.
  {
    // copy data to picture
    if (cpyPred)
    {
      getPredBuf(clippedArea).copyFrom(subPredBuf);
    }
    if (cpyResi)
    {
      getResiBuf(clippedArea).copyFrom(subResiBuf);
    }
    if (cpyReco)
    {
      getRecoBuf(clippedArea).copyFrom(subRecoBuf);
    }
    if (cpyOrgResi)
    {
      getOrgResiBuf(clippedArea).copyFrom(subStruct.getOrgResiBuf(clippedArea));
    }
  }

  if (cpyPred)  // Se cpyPred e cpyResi forem verdadeiros, copia os buffers para os buffers correspondentes no picture associado.
  {
    picture->getPredBuf(clippedArea).copyFrom(subPredBuf);
  }
  if (cpyResi)
  {
    picture->getResiBuf(clippedArea).copyFrom(subResiBuf);
  }
  if (cpyReco)
  {
    picture->getRecoBuf(clippedArea).copyFrom(subRecoBuf);
  }

  // Se a subestrutura não estiver sendo usada para a codificação de TUs e não for um canal cromático, copia os buffers de movimento.
  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->getSPS()->getIBCFlag()) && chType != CHANNEL_TYPE_CHROMA))
  {
    // copy motion buffer
    MotionBuf ownMB  = getMotionBuf          ( clippedArea );
    CMotionBuf subMB = subStruct.getMotionBuf( clippedArea );

    ownMB.copyFrom( subMB );

    motionLut = subStruct.motionLut;
  }
  prevPLT = subStruct.prevPLT;  // Copia a tabela de paletas anteriores.


  if ( updateCost )  // Atualiza as variáveis de custo, se necessário.
  {
    fracBits += subStruct.fracBits;
    dist     += subStruct.dist;
    cost     += subStruct.cost;
    costDbOffset += subStruct.costDbOffset;
  }
  if( parent )  // Verifica se a subestrutura está contida dentro da instância atual, a menos que seja a estrutura de nível superior.
  {
    // allow this to be false at the top level
    CHECKD( !area.contains( subArea ), "Trying to use a sub-structure not contained in self" );
  }

  // copy the CUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &pcu : subStruct.cus )  // Copia CUs, PUs e TUs da subestrutura para a instância atual.
    {
      // add an analogue CU into own CU store
      const UnitArea &cuPatch = *pcu;
      CodingUnit &cu = addCU( cuPatch, pcu->chType );

      // copy the CU info from subPatch
      cu = *pcu;
    }
  }

  // copy the PUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &ppu : subStruct.pus )
    {
      // add an analogue PU into own PU store
      const UnitArea &puPatch = *ppu;
      PredictionUnit &pu = addPU( puPatch, ppu->chType );

      // copy the PU info from subPatch
      pu = *ppu;
    }
  }
  // copy the TUs over
  for( const auto &ptu : subStruct.tus )
  {
    // add an analogue TU into own TU store
    const UnitArea &tuPatch = *ptu;
    TransformUnit &tu = addTU( tuPatch, ptu->chType );

    // copy the TU info from subPatch
    tu = *ptu;
  }
}

void CodingStructure::copyStructure( const CodingStructure& other, const ChannelType chType, const bool copyTUs, const bool copyRecoBuf )
{
  fracBits = other.fracBits;
  dist     = other.dist;
  cost     = other.cost;
  costDbOffset = other.costDbOffset;
  CHECKD( area != other.area, "Incompatible sizes" );

  const UnitArea dualITreeArea = CS::getArea( *this, this->area, chType );

  // copy the CUs over
  for (const auto &pcu : other.cus)
  {
    if( !dualITreeArea.contains( *pcu ) )
    {
      continue;
    }
    // add an analogue CU into own CU store
    const UnitArea &cuPatch = *pcu;

    CodingUnit &cu = addCU(cuPatch, pcu->chType);

    // copy the CU info from subPatch
    cu = *pcu;
  }

  // copy the PUs over
  for (const auto &ppu : other.pus)
  {
    if( !dualITreeArea.contains( *ppu ) )
    {
      continue;
    }
    // add an analogue PU into own PU store
    const UnitArea &puPatch = *ppu;

    PredictionUnit &pu = addPU(puPatch, ppu->chType);
    // copy the PU info from subPatch
    pu = *ppu;
  }

  if (!other.slice->isIntra() || other.slice->getSPS()->getIBCFlag())
  {
    // copy motion buffer
    MotionBuf  ownMB = getMotionBuf();
    CMotionBuf subMB = other.getMotionBuf();

    ownMB.copyFrom( subMB );

    motionLut = other.motionLut;
  }
  prevPLT = other.prevPLT;

  if( copyTUs )
  {
    // copy the TUs over
    for( const auto &ptu : other.tus )
    {
      if( !dualITreeArea.contains( *ptu ) )
      {
        continue;
      }
      // add an analogue TU into own TU store
      const UnitArea &tuPatch = *ptu;
      TransformUnit &tu = addTU( tuPatch, ptu->chType );
      // copy the TU info from subPatch
      tu = *ptu;
    }
  }

  if( copyRecoBuf )
  {
    CPelUnitBuf recoBuf = other.getRecoBuf( area );

    if( parent )
    {
      // copy data to self for neighbors
      getRecoBuf( area ).copyFrom( recoBuf );
    }

    // copy data to picture
    picture->getRecoBuf( area ).copyFrom( recoBuf );
    if (other.pcv->isEncoder)
    {
      CPelUnitBuf predBuf = other.getPredBuf(area);
      if (parent)
      {
        getPredBuf(area).copyFrom(predBuf);
      }
      picture->getPredBuf(area).copyFrom(predBuf);
    }

    // required for DebugCTU
    int numCh = ::getNumberValidChannels( area.chromaFormat );
    for( int i = 0; i < numCh; i++ )
    {
      const size_t _area = unitScale[i].scaleArea( area.blocks[i].area() );

      memcpy( m_isDecomp[i], other.m_isDecomp[i], sizeof( *m_isDecomp[0] ) * _area );
    }
  }
}

void CodingStructure::initStructData( const int &QP, const bool &skipMotBuf )
{
  clearPUs();
  clearTUs();
  clearCUs();

  if( QP < MAX_INT )
  {
    currQP[0] = currQP[1] = QP;
  }

  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->getSPS()->getIBCFlag()) && !m_isTuEnc)))
  {
    getMotionBuf().memset(0);
  }

  fracBits = 0;
  dist     = 0;
  cost     = MAX_DOUBLE;
  lumaCost = MAX_DOUBLE;
  costDbOffset = 0;
  useDbCost = false;
  interHad = std::numeric_limits<Distortion>::max();
}


void CodingStructure::clearTUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    size_t _area = ( area.blocks[i].area() >> unitScale[i].area );

    memset( m_isDecomp[i], false, sizeof( *m_isDecomp[0] ) * _area );
    memset( m_tuIdx   [i],     0, sizeof( *m_tuIdx   [0] ) * _area );
  }

  numCh = getNumberValidComponents( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    m_offsets[i] = 0;
  }

  for( auto &pcu : cus )
  {
    pcu->firstTU = pcu->lastTU = nullptr;
  }

  m_tuCache.cache( tus );
  m_numTUs = 0;
}

void CodingStructure::clearPUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_puIdx[i], 0, sizeof( *m_puIdx[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  m_puCache.cache( pus );
  m_numPUs = 0;

  for( auto &pcu : cus )
  {
    pcu->firstPU = pcu->lastPU = nullptr;
  }
}

void CodingStructure::clearCUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_cuIdx[i], 0, sizeof( *m_cuIdx[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  m_cuCache.cache( cus );
  m_numCUs = 0;
}

MotionBuf CodingStructure::getMotionBuf( const Area& _area )
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

const CMotionBuf CodingStructure::getMotionBuf( const Area& _area ) const
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

MotionInfo& CodingStructure::getMotionInfo( const Position& pos )
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}

const MotionInfo& CodingStructure::getMotionInfo( const Position& pos ) const
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}


// data accessors
       PelBuf     CodingStructure::getPredBuf(const CompArea &blk)           { return getBuf(blk,  PIC_PREDICTION); }
const CPelBuf     CodingStructure::getPredBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_PREDICTION); }
       PelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit)          { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_PREDICTION); }

       PelBuf     CodingStructure::getResiBuf(const CompArea &blk)           { return getBuf(blk,  PIC_RESIDUAL); }
const CPelBuf     CodingStructure::getResiBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_RESIDUAL); }
       PelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit)          { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_RESIDUAL); }

       PelBuf     CodingStructure::getRecoBuf(const CompArea &blk)           { return getBuf(blk,  PIC_RECONSTRUCTION); }
const CPelBuf     CodingStructure::getRecoBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_RECONSTRUCTION); }
       PelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)          { return getBuf(unit, PIC_RECONSTRUCTION); }
const CPelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_RECONSTRUCTION); }

       PelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk)        { return getBuf(blk,  PIC_ORG_RESI); }
const CPelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_ORG_RESI); }
       PelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit)       { return getBuf(unit, PIC_ORG_RESI); }
const CPelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORG_RESI); }

       PelBuf     CodingStructure::getOrgBuf(const CompArea &blk)            { return getBuf(blk,  PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const CompArea &blk)      const { return getBuf(blk,  PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit)           { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit)     const { return getBuf(unit, PIC_ORIGINAL); }

       PelBuf     CodingStructure::getOrgBuf(const ComponentID &compID)      { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const ComponentID &compID)const { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getOrgBuf()                               { return picture->getBuf(area, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf()                         const { return picture->getBuf(area, PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getTrueOrgBuf()                           { return picture->getBuf(area, PIC_TRUE_ORIGINAL); }
const CPelUnitBuf CodingStructure::getTrueOrgBuf()                     const { return picture->getBuf(area, PIC_TRUE_ORIGINAL); }

PelBuf CodingStructure::getBuf( const CompArea &blk, const PictureType &type )
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const ComponentID compID = blk.compID;

  PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : ( type == PIC_ORG_RESI ? &m_orgr : nullptr ) ) );

  CHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

#if !KEEP_PRED_AND_RESI_SIGNALS
  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
  {
    cFinal.x &= ( pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }
#endif

  return buf->getBuf( cFinal );
}

const CPelBuf CodingStructure::getBuf( const CompArea &blk, const PictureType &type ) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const ComponentID compID = blk.compID;

  const PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : ( type == PIC_ORG_RESI ? &m_orgr : nullptr ) ) );

  CHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

#if !KEEP_PRED_AND_RESI_SIGNALS
  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
  {
    cFinal.x &= ( pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }
#endif

  return buf->getBuf( cFinal );
}

PelUnitBuf CodingStructure::getBuf( const UnitArea &unit, const PictureType &type )
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf CodingStructure::getBuf( const UnitArea &unit, const PictureType &type ) const
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const CodingUnit& curCu, const ChannelType _chType ) const
{
  const CodingUnit* cu = getCU( pos, _chType );
  // exists       same slice and tile                  cu precedes curCu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  const bool wavefrontsEnabled = curCu.slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(curCu.cs->sps->getMaxCUWidth());
  int xNbY  = pos.x << getChannelTypeScaleX( _chType, curCu.chromaFormat );
  int xCurr = curCu.blocks[_chType].x << getChannelTypeScaleX( _chType, curCu.chromaFormat );
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;
  if( cu && CU::isSameSliceAndTile( *cu, curCu ) && ( cu->cs != curCu.cs || cu->idx <= curCu.idx ) && addCheck)
  {
    return cu;
  }
  else
  {
    return nullptr;
  }
}

const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType ) const
{
  const CodingUnit* cu = getCU( pos, _chType );
  const bool wavefrontsEnabled = this->slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(this->sps->getMaxCUWidth());

  const int xNbY  = pos.x * (1 << getChannelTypeScaleX(_chType, this->area.chromaFormat));
  const int xCurr = curPos.x * (1 << getChannelTypeScaleX(_chType, this->area.chromaFormat));

  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;
  return ( cu && cu->slice->getIndependentSliceIdx() == curSliceIdx && cu->tileIdx == curTileIdx && addCheck ) ? cu : nullptr;
}

const PredictionUnit* CodingStructure::getPURestricted( const Position &pos, const PredictionUnit& curPu, const ChannelType _chType ) const
{
  const PredictionUnit* pu = getPU( pos, _chType );
  // exists       same slice and tile                  pu precedes curPu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  const bool wavefrontsEnabled = curPu.cu->slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(curPu.cs->sps->getMaxCUWidth());
  int xNbY  = pos.x << getChannelTypeScaleX( _chType, curPu.chromaFormat );
  int xCurr = curPu.blocks[_chType].x << getChannelTypeScaleX( _chType, curPu.chromaFormat );
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;
  if( pu && CU::isSameSliceAndTile( *pu->cu, *curPu.cu ) && ( pu->cs != curPu.cs || pu->idx <= curPu.idx ) && addCheck )
  {
    return pu;
  }
  else
  {
    return nullptr;
  }
}

const TransformUnit* CodingStructure::getTURestricted( const Position &pos, const TransformUnit& curTu, const ChannelType _chType ) const
{
  const TransformUnit* tu = getTU( pos, _chType );
  // exists       same slice and tile                  tu precedes curTu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  const bool wavefrontsEnabled = curTu.cu->slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(curTu.cs->sps->getMaxCUWidth());
  int xNbY  = pos.x << getChannelTypeScaleX( _chType, curTu.chromaFormat );
  int xCurr = curTu.blocks[_chType].x << getChannelTypeScaleX( _chType, curTu.chromaFormat );
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;
  if( tu && CU::isSameSliceAndTile( *tu->cu, *curTu.cu ) && ( tu->cs != curTu.cs || tu->idx <= curTu.idx ) && addCheck )
  {
    return tu;
  }
  else
  {
    return nullptr;
  }
}


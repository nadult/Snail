#include <GL/gl.h>
#include <GL/glu.h>
#include "rtbase.h"
#include "triangle.h"
#include "gl_window.h"
#include <algorithm>
#include <set>
#include <stdio.h>


class Eigen
{
public:
    Eigen (int iSize);
    ~Eigen ();

    // set the matrix for eigensolving
    float& Matrix (int iRow, int iCol) { return m_aafMat[iRow][iCol]; }
    void SetMatrix (float** aafMat);

    // get the results of eigensolving (eigenvectors are columns of matrix)
    float GetEigenvalue (int i) const { return m_afDiag[i]; }
    float GetEigenvector (int iRow, int iCol) const { return m_aafMat[iRow][iCol]; }
    float* GetEigenvalue () { return m_afDiag; }
    float** GetEigenvector () { return m_aafMat; }

    // solve eigensystem
    void EigenStuff2 ();
    void EigenStuff3 ();
    void EigenStuff4 ();
    void EigenStuffN ();
    void EigenStuff  ();

    // solve eigensystem, use decreasing sort on eigenvalues
    void DecrSortEigenStuff2 ();
    void DecrSortEigenStuff3 ();
    void DecrSortEigenStuff4 ();
    void DecrSortEigenStuffN ();
    void DecrSortEigenStuff  ();

    // solve eigensystem, use increasing sort on eigenvalues
    void IncrSortEigenStuff2 ();
    void IncrSortEigenStuff3 ();
    void IncrSortEigenStuff4 ();
    void IncrSortEigenStuffN ();
    void IncrSortEigenStuff  ();

protected:
    int m_iSize;
    float** m_aafMat;
    float* m_afDiag;
    float* m_afSubd;

    // Householder reduction to tridiagonal form
    static void Tridiagonal2 (float** aafMat, float* afDiag, float* afSubd);
    static void Tridiagonal3 (float** aafMat, float* afDiag, float* afSubd);
    static void Tridiagonal4 (float** aafMat, float* afDiag, float* afSubd);
    static void TridiagonalN (int iSize, float** aafMat, float* afDiag,
        float* afSubd);

    // QL algorithm with implicit shifting, applies to tridiagonal matrices
    static bool QLAlgorithm (int iSize, float* afDiag, float* afSubd,
        float** aafMat);

    // sort eigenvalues from largest to smallest
    static void DecreasingSort (int iSize, float* afEigval,
        float** aafEigvec);

    // sort eigenvalues from smallest to largest
    static void IncreasingSort (int iSize, float* afEigval,
        float** aafEigvec);
};

Eigen::Eigen (int iSize)
{
    assert( iSize >= 2 );
    m_iSize = iSize;

    m_aafMat = new float*[m_iSize];
    for (int i = 0; i < m_iSize; i++)
        m_aafMat[i] = new float[m_iSize];

    m_afDiag = new float[m_iSize];
    m_afSubd = new float[m_iSize];
}
Eigen::~Eigen ()
{
    delete[] m_afSubd;
    delete[] m_afDiag;
    for (int i = 0; i < m_iSize; i++)
        delete[] m_aafMat[i];
    delete[] m_aafMat;
}
void Eigen::Tridiagonal2 (float** m_aafMat, float* m_afDiag, float* m_afSubd)
{
    // matrix is already tridiagonal
    m_afDiag[0] = m_aafMat[0][0];
    m_afDiag[1] = m_aafMat[1][1];
    m_afSubd[0] = m_aafMat[0][1];
    m_afSubd[1] = 0.0f;
    m_aafMat[0][0] = 1.0f;
    m_aafMat[0][1] = 0.0f;
    m_aafMat[1][0] = 0.0f;
    m_aafMat[1][1] = 1.0f;
}
void Eigen::Tridiagonal3 (float** m_aafMat, float* m_afDiag, float* m_afSubd)
{
    float fM00 = m_aafMat[0][0];
    float fM01 = m_aafMat[0][1];
    float fM02 = m_aafMat[0][2];
    float fM11 = m_aafMat[1][1];
    float fM12 = m_aafMat[1][2];
    float fM22 = m_aafMat[2][2];

    m_afDiag[0] = fM00;
    m_afSubd[2] = 0.0f;
    if ( fM02 != 0.0f )
    {
        float fLength = Sqrt(fM01*fM01+fM02*fM02);
        float fInvLength = 1.0f/fLength;
        fM01 *= fInvLength;
        fM02 *= fInvLength;
        float fQ = 2.0f*fM01*fM12+fM02*(fM22-fM11);
        m_afDiag[1] = fM11+fM02*fQ;
        m_afDiag[2] = fM22-fM02*fQ;
        m_afSubd[0] = fLength;
        m_afSubd[1] = fM12-fM01*fQ;
        m_aafMat[0][0] = 1.0f; m_aafMat[0][1] = 0.0f;  m_aafMat[0][2] = 0.0f;
        m_aafMat[1][0] = 0.0f; m_aafMat[1][1] = fM01; m_aafMat[1][2] = fM02;
        m_aafMat[2][0] = 0.0f; m_aafMat[2][1] = fM02; m_aafMat[2][2] = -fM01;
    }
    else
    {
        m_afDiag[1] = fM11;
        m_afDiag[2] = fM22;
        m_afSubd[0] = fM01;
        m_afSubd[1] = fM12;
        m_aafMat[0][0] = 1.0f; m_aafMat[0][1] = 0.0f; m_aafMat[0][2] = 0.0f;
        m_aafMat[1][0] = 0.0f; m_aafMat[1][1] = 1.0f; m_aafMat[1][2] = 0.0f;
        m_aafMat[2][0] = 0.0f; m_aafMat[2][1] = 0.0f; m_aafMat[2][2] = 1.0f;
    }
}
void Eigen::Tridiagonal4 (float** m_aafMat, float* m_afDiag, float* m_afSubd)
{
    // save matrix M
    float fM00 = m_aafMat[0][0];
    float fM01 = m_aafMat[0][1];
    float fM02 = m_aafMat[0][2];
    float fM03 = m_aafMat[0][3];
    float fM11 = m_aafMat[1][1];
    float fM12 = m_aafMat[1][2];
    float fM13 = m_aafMat[1][3];
    float fM22 = m_aafMat[2][2];
    float fM23 = m_aafMat[2][3];
    float fM33 = m_aafMat[3][3];

    m_afDiag[0] = fM00;
    m_afSubd[3] = 0.0f;

    m_aafMat[0][0] = 1.0f;
    m_aafMat[0][1] = 0.0f;
    m_aafMat[0][2] = 0.0f;
    m_aafMat[0][3] = 0.0f;
    m_aafMat[1][0] = 0.0f;
    m_aafMat[2][0] = 0.0f;
    m_aafMat[3][0] = 0.0f;

    float fLength, fInvLength;

    if ( fM02 != 0.0f || fM03 != 0.0f )
    {
        float fQ11, fQ12, fQ13;
        float fQ21, fQ22, fQ23;
        float fQ31, fQ32, fQ33;

        // build column Q1
        fLength = Sqrt(fM01*fM01+fM02*fM02+fM03*fM03);
        fInvLength = 1.0f/fLength;
        fQ11 = fM01*fInvLength;
        fQ21 = fM02*fInvLength;
        fQ31 = fM03*fInvLength;

        m_afSubd[0] = fLength;

        // compute S*Q1
        float fV0 = fM11*fQ11+fM12*fQ21+fM13*fQ31;
        float fV1 = fM12*fQ11+fM22*fQ21+fM23*fQ31;
        float fV2 = fM13*fQ11+fM23*fQ21+fM33*fQ31;

        m_afDiag[1] = fQ11*fV0+fQ21*fV1+fQ31*fV2;

        // build column Q3 = Q1x(S*Q1)
        fQ13 = fQ21*fV2-fQ31*fV1;
        fQ23 = fQ31*fV0-fQ11*fV2;
        fQ33 = fQ11*fV1-fQ21*fV0;
        fLength = Sqrt(fQ13*fQ13+fQ23*fQ23+fQ33*fQ33);
        if ( fLength > 0.0f )
        {
            fInvLength = 1.0f/fLength;
            fQ13 *= fInvLength;
            fQ23 *= fInvLength;
            fQ33 *= fInvLength;

            // build column Q2 = Q3xQ1
            fQ12 = fQ23*fQ31-fQ33*fQ21; 
            fQ22 = fQ33*fQ11-fQ13*fQ31;
            fQ32 = fQ13*fQ21-fQ23*fQ11;

            fV0 = fQ12*fM11+fQ22*fM12+fQ32*fM13;
            fV1 = fQ12*fM12+fQ22*fM22+fQ32*fM23;
            fV2 = fQ12*fM13+fQ22*fM23+fQ32*fM33;
            m_afSubd[1] = fQ11*fV0+fQ21*fV1+fQ31*fV2;
            m_afDiag[2] = fQ12*fV0+fQ22*fV1+fQ32*fV2;
            m_afSubd[2] = fQ13*fV0+fQ23*fV1+fQ33*fV2;

            fV0 = fQ13*fM11+fQ23*fM12+fQ33*fM13;
            fV1 = fQ13*fM12+fQ23*fM22+fQ33*fM23;
            fV2 = fQ13*fM13+fQ23*fM23+fQ33*fM33;
            m_afDiag[3] = fQ13*fV0+fQ23*fV1+fQ33*fV2;
        }
        else
        {
            // S*Q1 parallel to Q1, choose any valid Q2 and Q3
            m_afSubd[1] = 0.0f;

            fLength = fQ21*fQ21+fQ31*fQ31;
            if ( fLength > 0.0f )
            {
                fInvLength = 1.0f/fLength;
                float fTmp = fQ11-1.0f;
                fQ12 = -fQ21;
                fQ22 = 1.0f+fTmp*fQ21*fQ21*fInvLength;
                fQ32 = fTmp*fQ21*fQ31*fInvLength;

                fQ13 = -fQ31;
                fQ23 = fQ32;
                fQ33 = 1.0f+fTmp*fQ31*fQ31*fInvLength;

                fV0 = fQ12*fM11+fQ22*fM12+fQ32*fM13;
                fV1 = fQ12*fM12+fQ22*fM22+fQ32*fM23;
                fV2 = fQ12*fM13+fQ22*fM23+fQ32*fM33;
                m_afDiag[2] = fQ12*fV0+fQ22*fV1+fQ32*fV2;
                m_afSubd[2] = fQ13*fV0+fQ23*fV1+fQ33*fV2;

                fV0 = fQ13*fM11+fQ23*fM12+fQ33*fM13;
                fV1 = fQ13*fM12+fQ23*fM22+fQ33*fM23;
                fV2 = fQ13*fM13+fQ23*fM23+fQ33*fM33;
                m_afDiag[3] = fQ13*fV0+fQ23*fV1+fQ33*fV2;
            }
            else
            {
                // Q1 = (+-1,0,0)
                fQ12 = 0.0f; fQ22 = 1.0f; fQ32 = 0.0f;
                fQ13 = 0.0f; fQ23 = 0.0f; fQ33 = 1.0f;

                m_afDiag[2] = fM22;
                m_afDiag[3] = fM33;
                m_afSubd[2] = fM23;
            }
        }

        m_aafMat[1][1] = fQ11; m_aafMat[1][2] = fQ12; m_aafMat[1][3] = fQ13;
        m_aafMat[2][1] = fQ21; m_aafMat[2][2] = fQ22; m_aafMat[2][3] = fQ23;
        m_aafMat[3][1] = fQ31; m_aafMat[3][2] = fQ32; m_aafMat[3][3] = fQ33;
    }
    else
    {
        m_afDiag[1] = fM11;
        m_afSubd[0] = fM01;
        m_aafMat[1][1] = 1.0f;
        m_aafMat[2][1] = 0.0f;
        m_aafMat[3][1] = 0.0f; 

        if ( fM13 != 0.0f )
        {
            fLength = Sqrt(fM12*fM12+fM13*fM13);
            fInvLength = 1.0f/fLength;
            fM12 *= fInvLength;
            fM13 *= fInvLength;
            float fQ = 2.0f*fM12*fM23+fM13*(fM33-fM22);

            m_afDiag[2] = fM22+fM13*fQ;
            m_afDiag[3] = fM33-fM13*fQ;
            m_afSubd[1] = fLength;
            m_afSubd[2] = fM23-fM12*fQ;
            m_aafMat[1][2] = 0.0f;
            m_aafMat[1][3] = 0.0f;
            m_aafMat[2][2] = fM12;
            m_aafMat[2][3] = fM13;
            m_aafMat[3][2] = fM13;
            m_aafMat[3][3] = -fM12;
        }
        else
        {
            m_afDiag[2] = fM22;
            m_afDiag[3] = fM33;
            m_afSubd[1] = fM12;
            m_afSubd[2] = fM23;
            m_aafMat[1][2] = 0.0f;
            m_aafMat[1][3] = 0.0f;
            m_aafMat[2][2] = 1.0f;
            m_aafMat[2][3] = 0.0f;
            m_aafMat[3][2] = 0.0f;
            m_aafMat[3][3] = 1.0f;
        }
    }
}
void Eigen::TridiagonalN (int iSize, float** m_aafMat, float* m_afDiag,
    float* m_afSubd)
{
    int i0, i1, i2, i3;

    for (i0 = iSize-1, i3 = iSize-2; i0 >= 1; i0--, i3--)
    {
        float fH = 0.0f, fScale = 0.0f;

        if ( i3 > 0 )
        {
            for (i2 = 0; i2 <= i3; i2++)
                fScale += Abs(m_aafMat[i0][i2]);
            if ( fScale == 0.0f )
            {
                m_afSubd[i0] = m_aafMat[i0][i3];
            }
            else
            {
                float fInvScale = 1.0f/fScale;
                for (i2 = 0; i2 <= i3; i2++)
                {
                    m_aafMat[i0][i2] *= fInvScale;
                    fH += m_aafMat[i0][i2]*m_aafMat[i0][i2];
                }
                float fF = m_aafMat[i0][i3];
                float fG = Sqrt(fH);
                if ( fF > 0.0f )
                    fG = -fG;
                m_afSubd[i0] = fScale*fG;
                fH -= fF*fG;
                m_aafMat[i0][i3] = fF-fG;
                fF = 0.0f;
                float fInvH = 1.0f/fH;
                for (i1 = 0; i1 <= i3; i1++)
                {
                    m_aafMat[i1][i0] = m_aafMat[i0][i1]*fInvH;
                    fG = 0.0f;
                    for (i2 = 0; i2 <= i1; i2++)
                        fG += m_aafMat[i1][i2]*m_aafMat[i0][i2];
                    for (i2 = i1+1; i2 <= i3; i2++)
                        fG += m_aafMat[i2][i1]*m_aafMat[i0][i2];
                    m_afSubd[i1] = fG*fInvH;
                    fF += m_afSubd[i1]*m_aafMat[i0][i1];
                }
                float fHalfFdivH = 0.5f*fF*fInvH;
                for (i1 = 0; i1 <= i3; i1++)
                {
                    fF = m_aafMat[i0][i1];
                    fG = m_afSubd[i1] - fHalfFdivH*fF;
                    m_afSubd[i1] = fG;
                    for (i2 = 0; i2 <= i1; i2++)
                    {
                        m_aafMat[i1][i2] -= fF*m_afSubd[i2] +
                            fG*m_aafMat[i0][i2];
                    }
                }
            }
        }
        else
        {
            m_afSubd[i0] = m_aafMat[i0][i3];
        }

        m_afDiag[i0] = fH;
    }

    m_afDiag[0] = 0.0f;
    m_afSubd[0] = 0.0f;
    for (i0 = 0, i3 = -1; i0 <= iSize-1; i0++, i3++)
    {
        if ( m_afDiag[i0] != 0.0f )
        {
            for (i1 = 0; i1 <= i3; i1++)
            {
                float fSum = 0.0f;
                for (i2 = 0; i2 <= i3; i2++)
                    fSum += m_aafMat[i0][i2]*m_aafMat[i2][i1];
                for (i2 = 0; i2 <= i3; i2++)
                    m_aafMat[i2][i1] -= fSum*m_aafMat[i2][i0];
            }
        }
        m_afDiag[i0] = m_aafMat[i0][i0];
        m_aafMat[i0][i0] = 1.0f;
        for (i1 = 0; i1 <= i3; i1++)
        {
            m_aafMat[i1][i0] = 0.0f;
            m_aafMat[i0][i1] = 0.0f;
        }
    }

    // re-ordering if Eigen::QLAlgorithm is used subsequently
    for (i0 = 1, i3 = 0; i0 < iSize; i0++, i3++)
        m_afSubd[i3] = m_afSubd[i0];
    m_afSubd[iSize-1] = 0.0f;
}
bool Eigen::QLAlgorithm (int iSize, float* m_afDiag, float* m_afSubd,
    float** m_aafMat)
{
    const int iMaxIter = 32;

    for (int i0 = 0; i0 < iSize; i0++)
    {
        int i1;
        for (i1 = 0; i1 < iMaxIter; i1++)
        {
            int i2;
            for (i2 = i0; i2 <= iSize-2; i2++)
            {
                float fTmp = Abs(m_afDiag[i2]) +
                    Abs(m_afDiag[i2+1]);
                if ( Abs(m_afSubd[i2]) + fTmp == fTmp )
                    break;
            }
            if ( i2 == i0 )
                break;

            float fG = (m_afDiag[i0+1]-m_afDiag[i0])/(2.0f*m_afSubd[i0]);
            float fR = Sqrt(fG*fG+1.0f);
            if ( fG < 0.0f )
                fG = m_afDiag[i2]-m_afDiag[i0]+m_afSubd[i0]/(fG-fR);
            else
                fG = m_afDiag[i2]-m_afDiag[i0]+m_afSubd[i0]/(fG+fR);
            float fSin = 1.0f, fCos = 1.0f, fP = 0.0f;
            for (int i3 = i2-1; i3 >= i0; i3--)
            {
                float fF = fSin*m_afSubd[i3];
                float fB = fCos*m_afSubd[i3];
                if ( Abs(fF) >= Abs(fG) )
                {
                    fCos = fG/fF;
                    fR = Sqrt(fCos*fCos+1.0f);
                    m_afSubd[i3+1] = fF*fR;
                    fSin = 1.0f/fR;
                    fCos *= fSin;
                }
                else
                {
                    fSin = fF/fG;
                    fR = Sqrt(fSin*fSin+1.0f);
                    m_afSubd[i3+1] = fG*fR;
                    fCos = 1.0f/fR;
                    fSin *= fCos;
                }
                fG = m_afDiag[i3+1]-fP;
                fR = (m_afDiag[i3]-fG)*fSin+2.0f*fB*fCos;
                fP = fSin*fR;
                m_afDiag[i3+1] = fG+fP;
                fG = fCos*fR-fB;

                for (int i4 = 0; i4 < iSize; i4++)
                {
                    fF = m_aafMat[i4][i3+1];
                    m_aafMat[i4][i3+1] = fSin*m_aafMat[i4][i3]+fCos*fF;
                    m_aafMat[i4][i3] = fCos*m_aafMat[i4][i3]-fSin*fF;
                }
            }
            m_afDiag[i0] -= fP;
            m_afSubd[i0] = fG;
            m_afSubd[i2] = 0.0f;
        }
        if ( i1 == iMaxIter )
            return false;
    }

    return true;
}
void Eigen::DecreasingSort (int iSize, float* afEigval, float** aafEigvec)
{
    // sort eigenvalues in decreasing order, e[0] >= ... >= e[iSize-1]
    for (int i0 = 0, i1; i0 <= iSize-2; i0++)
    {
        // locate maximum eigenvalue
        i1 = i0;
        float fMax = afEigval[i1];
        int i2;
        for (i2 = i0+1; i2 < iSize; i2++)
        {
            if ( afEigval[i2] > fMax )
            {
                i1 = i2;
                fMax = afEigval[i1];
            }
        }

        if ( i1 != i0 )
        {
            // swap eigenvalues
            afEigval[i1] = afEigval[i0];
            afEigval[i0] = fMax;

            // swap eigenvectors
            for (i2 = 0; i2 < iSize; i2++)
            {
                float fTmp = aafEigvec[i2][i0];
                aafEigvec[i2][i0] = aafEigvec[i2][i1];
                aafEigvec[i2][i1] = fTmp;
            }
        }
    }
}
void Eigen::IncreasingSort (int iSize, float* afEigval, float** aafEigvec)
{
    // sort eigenvalues in increasing order, e[0] <= ... <= e[iSize-1]
    for (int i0 = 0, i1; i0 <= iSize-2; i0++)
    {
        // locate minimum eigenvalue
        i1 = i0;
        float fMin = afEigval[i1];
        int i2;
        for (i2 = i0+1; i2 < iSize; i2++)
        {
            if ( afEigval[i2] < fMin )
            {
                i1 = i2;
                fMin = afEigval[i1];
            }
        }

        if ( i1 != i0 )
        {
            // swap eigenvalues
            afEigval[i1] = afEigval[i0];
            afEigval[i0] = fMin;

            // swap eigenvectors
            for (i2 = 0; i2 < iSize; i2++)
            {
                float fTmp = aafEigvec[i2][i0];
                aafEigvec[i2][i0] = aafEigvec[i2][i1];
                aafEigvec[i2][i1] = fTmp;
            }
        }
    }
}
void Eigen::SetMatrix (float** aafMat)
{
    for (int iRow = 0; iRow < m_iSize; iRow++)
    {
        for (int iCol = 0; iCol < m_iSize; iCol++)
            m_aafMat[iRow][iCol] = aafMat[iRow][iCol];
    }
}
void Eigen::EigenStuff2 ()
{
    Tridiagonal2(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
}
void Eigen::EigenStuff3 ()
{
    Tridiagonal3(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
}
void Eigen::EigenStuff4 ()
{
    Tridiagonal4(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
}
void Eigen::EigenStuffN ()
{
    TridiagonalN(m_iSize,m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
}
void Eigen::EigenStuff ()
{
    switch ( m_iSize )
    {
        case 2:
            Tridiagonal2(m_aafMat,m_afDiag,m_afSubd);
            break;
        case 3:
            Tridiagonal3(m_aafMat,m_afDiag,m_afSubd);
            break;
        case 4:
            Tridiagonal4(m_aafMat,m_afDiag,m_afSubd);
            break;
        default:
            TridiagonalN(m_iSize,m_aafMat,m_afDiag,m_afSubd);
            break;
    }
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
}
void Eigen::DecrSortEigenStuff2 ()
{
    Tridiagonal2(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    DecreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::DecrSortEigenStuff3 ()
{
    Tridiagonal3(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    DecreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::DecrSortEigenStuff4 ()
{
    Tridiagonal4(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    DecreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::DecrSortEigenStuffN ()
{
    TridiagonalN(m_iSize,m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    DecreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::DecrSortEigenStuff ()
{
    switch ( m_iSize )
    {
        case 2:
            Tridiagonal2(m_aafMat,m_afDiag,m_afSubd);
            break;
        case 3:
            Tridiagonal3(m_aafMat,m_afDiag,m_afSubd);
            break;
        case 4:
            Tridiagonal4(m_aafMat,m_afDiag,m_afSubd);
            break;
        default:
            TridiagonalN(m_iSize,m_aafMat,m_afDiag,m_afSubd);
            break;
    }
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    DecreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::IncrSortEigenStuff2 ()
{
    Tridiagonal2(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    IncreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::IncrSortEigenStuff3 ()
{
    Tridiagonal3(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    IncreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::IncrSortEigenStuff4 ()
{
    Tridiagonal4(m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    IncreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::IncrSortEigenStuffN ()
{
    TridiagonalN(m_iSize,m_aafMat,m_afDiag,m_afSubd);
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    IncreasingSort(m_iSize,m_afDiag,m_aafMat);
}
void Eigen::IncrSortEigenStuff ()
{
    switch ( m_iSize )
    {
        case 2:
            Tridiagonal2(m_aafMat,m_afDiag,m_afSubd);
            break;
        case 3:
            Tridiagonal3(m_aafMat,m_afDiag,m_afSubd);
            break;
        case 4:
            Tridiagonal4(m_aafMat,m_afDiag,m_afSubd);
            break;
        default:
            TridiagonalN(m_iSize,m_aafMat,m_afDiag,m_afSubd);
            break;
    }
    QLAlgorithm(m_iSize,m_afDiag,m_afSubd,m_aafMat);
    IncreasingSort(m_iSize,m_afDiag,m_aafMat);
}


void GaussPointsFit (int iQuantity, const Vec3f* akPoint,Vec3f& rkCenter, Vec3f akAxis[3], float afExtent[3]) {
    // compute mean of points
    rkCenter = akPoint[0];
    int i;
    for (i = 1; i < iQuantity; i++)
        rkCenter += akPoint[i];
    float fInvQuantity = 1.0f/iQuantity;
    rkCenter *= fInvQuantity;

    // compute covariances of points
    float fSumXX = 0.0f, fSumXY = 0.0f, fSumXZ = 0.0f;
    float fSumYY = 0.0f, fSumYZ = 0.0f, fSumZZ = 0.0f;
    for (i = 0; i < iQuantity; i++)
    {
        Vec3f kDiff = akPoint[i] - rkCenter;
        fSumXX += kDiff.x*kDiff.x;
        fSumXY += kDiff.x*kDiff.y;
        fSumXZ += kDiff.x*kDiff.z;
        fSumYY += kDiff.y*kDiff.y;
        fSumYZ += kDiff.y*kDiff.z;
        fSumZZ += kDiff.z*kDiff.z;
    }
    fSumXX *= fInvQuantity;
    fSumXY *= fInvQuantity;
    fSumXZ *= fInvQuantity;
    fSumYY *= fInvQuantity;
    fSumYZ *= fInvQuantity;
    fSumZZ *= fInvQuantity;

    // compute eigenvectors for covariance matrix
    Eigen kES(3);
    kES.Matrix(0,0) = fSumXX;
    kES.Matrix(0,1) = fSumXY;
    kES.Matrix(0,2) = fSumXZ;
    kES.Matrix(1,0) = fSumXY;
    kES.Matrix(1,1) = fSumYY;
    kES.Matrix(1,2) = fSumYZ;
    kES.Matrix(2,0) = fSumXZ;
    kES.Matrix(2,1) = fSumYZ;
    kES.Matrix(2,2) = fSumZZ;
    kES.IncrSortEigenStuff3();

    akAxis[0].x = kES.GetEigenvector(0,0);
    akAxis[0].y = kES.GetEigenvector(1,0);
    akAxis[0].z = kES.GetEigenvector(2,0);
    akAxis[1].x = kES.GetEigenvector(0,1);
    akAxis[1].y = kES.GetEigenvector(1,1);
    akAxis[1].z = kES.GetEigenvector(2,1);
    akAxis[2].x = kES.GetEigenvector(0,2);
    akAxis[2].y = kES.GetEigenvector(1,2);
    akAxis[2].z = kES.GetEigenvector(2,2);

    afExtent[0] = kES.GetEigenvalue(0);
    afExtent[1] = kES.GetEigenvalue(1);
    afExtent[2] = kES.GetEigenvalue(2);
}

enum { resX=800, resY=600 };

typedef veclib::Matrix<Vec4f> Mat;

Mat RotationMatrix(const Vec3f &axis,float angle) {
	Vec4f mat[4];
	glPushMatrix();
	glRotatef(angle,axis.x,axis.y,axis.z);
	glGetFloatv(GL_MODELVIEW_MATRIX,&mat[0].x);
	glPopMatrix();

	return Mat(mat[0],mat[1],mat[2],mat[3]);
}

Vec3f RotateVecY(const Vec3f& v,float angle){
	return Vec3f(v.x*cos(angle)-v.z*sin(angle),v.y,v.x*sin(angle)+v.z*cos(angle));
}

Vec3f Unproject(float mx,float my,float depth) {
	double mat1[16],mat2[16];
	int viewport[4];
	
	my=resY-my;

	glGetDoublev(GL_MODELVIEW_MATRIX,mat1);
	glGetDoublev(GL_PROJECTION_MATRIX,mat2);
	glGetIntegerv(GL_VIEWPORT,viewport);
	double x,y,z;
	gluUnProject(mx,my,depth,mat1,mat2,viewport,&x,&y,&z);
	return Vec3f(x,y,z);
}

struct Camera {
	Camera() :pos(0,0,0),up(0,1,0),front(0,0,1) { }

	void Use() {
		glLoadIdentity();
		Vec3f a=pos,b=front+pos;
		gluLookAt(a.x,a.y,a.z,b.x,b.y,b.z,up.x,up.y,up.z);
	}
	void React(GLWindow &input) {
		float speed=25.0f;

		Vec3f right=RotateVecY(front,ConstPI<float>()*0.5f);
		if(input.Key('A')) pos-=right*speed;
		if(input.Key('D')) pos+=right*speed;
		if(input.Key('W')) pos+=front*speed;
		if(input.Key('S')) pos-=front*speed;
		if(input.Key('R')) pos+=up*speed;
		if(input.Key('F')) pos-=up*speed;

		if(input.MouseKey(2)) {
			Vec3f mv=input.MouseMove();
			if(Abs(mv.x)>0.5f) {
				front=RotateVecY(front,mv.x*0.05f);
				front*=RSqrt(front|front);
			}
		}
	}


	Vec3f pos,up,front;
};

struct Box {
	Box() { }
	Box(const Vec3f &min,const Vec3f &max) {
		center=(min+max)*0.5f;
		axis[0]=Vec3f(1,0,0);
		axis[1]=Vec3f(0,1,0);
		axis[2]=Vec3f(0,0,1);
		Vec3f size=(max-min)*0.5f;
		extent[0]=size.x;
		extent[1]=size.y;
		extent[2]=size.z;
	}
	Box(const Vec3f* points,int count) {
		GaussPointsFit(count,points,center,axis,extent);

		// Let C be the box center and let U0, U1, and U2 be the box axes.  Each
		// input point is of the form X = C + y0*U0 + y1*U1 + y2*U2.  The
		// following code computes min(y0), max(y0), min(y1), max(y1), min(y2),
		// and max(y2).  The box center is then adjusted to be
		//   C' = C + 0.5*(min(y0)+max(y0))*U0 + 0.5*(min(y1)+max(y1))*U1 +
		//        0.5*(min(y2)+max(y2))*U2

		Vec3f kDiff = points[0] - center;
		float fY0Min = kDiff|axis[0], fY0Max = fY0Min;
		float fY1Min = kDiff|axis[1], fY1Max = fY1Min;
		float fY2Min = kDiff|axis[2], fY2Max = fY2Min;

		for (int i = 1; i < count; i++) {
			kDiff = points[i] - center;

			float fY0=kDiff|axis[0];
			if(fY0<fY0Min ) fY0Min=fY0;
			else if(fY0>fY0Max) fY0Max=fY0;

			float fY1=kDiff|axis[1];
			if(fY1<fY1Min) fY1Min=fY1;
			else if(fY1>fY1Max) fY1Max=fY1;

			float fY2=kDiff|axis[2];
			if(fY2<fY2Min) fY2Min=fY2;
			else if(fY2>fY2Max) fY2Max=fY2;
		}

		center += (axis[0]*(fY0Min+fY0Max)+axis[1]*(fY1Min+fY1Max)+axis[2]*(fY2Min+fY2Max)) *0.5f;

		extent[0] = 0.5f*(fY0Max - fY0Min);
		extent[1] = 0.5f*(fY1Max - fY1Min);
		extent[2] = 0.5f*(fY2Max - fY2Min);
	}

	void GetPoints(Vec3f *points) const {
		Vec3f akEAxis[3] = { axis[0]*extent[0], axis[1]*extent[1], axis[2]*extent[2] };

		points[0] = center - akEAxis[0] - akEAxis[2] - akEAxis[1];
		points[1] = center + akEAxis[0] - akEAxis[2] - akEAxis[1];
		points[2] = center - akEAxis[0] + akEAxis[2] - akEAxis[1];
		points[3] = center + akEAxis[0] + akEAxis[2] - akEAxis[1];
		points[4] = center - akEAxis[0] - akEAxis[2] + akEAxis[1];
		points[5] = center + akEAxis[0] - akEAxis[2] + akEAxis[1];
		points[6] = center - akEAxis[0] + akEAxis[2] + akEAxis[1];
		points[7] = center + akEAxis[0] + akEAxis[2] + akEAxis[1];
	}

	bool Inside(const Vec3f &point,float fEpsilon=0.0001f) const {
		Vec3f kDiff=point-center;
		for (int i=0;i<3;i++) {
			float fCoeff=kDiff|axis[i];
			if (Abs(fCoeff)>extent[i]+fEpsilon) return false;
		}
		return true;
	}
	bool FullInside(const Triangle &tri) const {
		return Inside(tri.P1())&&Inside(tri.P2())&&Inside(tri.P3());
	}

	Vec3f center,axis[3];
	float extent[3];
};

float TriSurface(const Vec3f &a,const Vec3f &b,const Vec3f &c) {
	Vec3f e=c-a;
	float r=Sqrt(e|e);
	e/=r;

	float p=(b-a)|e;
	Vec3f hv=(a+e*p)-b;
	float h=Sqrt(hv|hv);

	return r*h*0.5f;
}

void glVertex(const Vec3f &v) { glVertex3f(v.x,v.y,v.z); }
void glVertex(float x,float y,float z) { glVertex3f(x,y,z); }
void glVertex(const Vec3p &v) { glVertex3f(v.x,v.y,v.z); }

void RenderTri(const Triangle &tri,Vec3f col) {
	glBegin(GL_TRIANGLES);
		glColor3f(col.x,col.y,col.z);
		glNormal3f(-tri.Nrm().x,-tri.Nrm().y,-tri.Nrm().z);
		glVertex(tri.P1());
		glVertex(tri.P2());
		glVertex(tri.P3());
	glEnd();
}

float QuadSurface(const Vec3f &a,const Vec3f &b,const Vec3f &c,const Vec3f &d) {
	return TriSurface(a,b,c)+TriSurface(a,c,d);
}

float BoxSurface(const Box &box) {
	Vec3f p[8]; box.GetPoints(p);

//	return 4.0f*( TriSurface(p[0],p[1],p[5]) + TriSurface(p[1],p[3],p[7]) + TriSurface(p[4],p[5],p[7]) );
	return  2.0f*(	QuadSurface(p[0],p[1],p[5],p[4])+
					QuadSurface(p[1],p[3],p[7],p[5])+
					QuadSurface(p[4],p[5],p[7],p[6]) );
}

void RenderBox(const Box &box,const Vec4f &col) {
	Vec3f p[8]; box.GetPoints(p);
	glBegin(GL_QUADS);
		glColor4f(col.x,col.y,col.z,col.w);
		glVertex(p[0]); glVertex(p[1]); glVertex(p[5]); glVertex(p[4]);
		glVertex(p[1]); glVertex(p[3]); glVertex(p[7]); glVertex(p[5]);
		glVertex(p[4]); glVertex(p[5]); glVertex(p[7]); glVertex(p[6]);
		glVertex(p[2]); glVertex(p[0]); glVertex(p[4]); glVertex(p[6]);
		glVertex(p[3]); glVertex(p[2]); glVertex(p[6]); glVertex(p[7]);
		glVertex(p[2]); glVertex(p[3]); glVertex(p[1]); glVertex(p[0]);
	glEnd();
}

void Render(TriVector &tris) {
	glBegin(GL_TRIANGLES);
	glColor3f(1,1,1);
	for(int n=0;n<tris.size();n++) {
		const Triangle &tri=tris[n];
		glNormal3f(-tri.Nrm().x,-tri.Nrm().y,-tri.Nrm().z);
		glVertex(tri.P1());
		glVertex(tri.P2());
		glVertex(tri.P3());
	}
	glEnd();
}

class TBox {
public:
	TBox() :tris(0) { }
	TBox(const TriVector *t) :tris(t),needUpdate(1) { }
	TBox(const TBox &b) :tris(b.tris),inds(b.inds) {
		needUpdate=0;
		b.Update();
		surface=b.surface;
		box=b.box;
	}

	float Surface() const { Update(); return surface; }
	Box GetBox() const { Update(); return box; }
	void Insert(int idx) { inds.insert(idx); }

private:
	void Update() const {
		if(!needUpdate) return;
		vector<Vec3f> points(inds.size()*3);

		int n=0;
		for(Iter it=inds.begin();it!=inds.end();++it) {
			const Triangle &tri=(*tris)[*it];
			points[n+0]=tri.P1();
			points[n+1]=tri.P2();
			points[n+2]=tri.P3();
			n+=3;
		}

		box=Box(&points[0],points.size());
		surface=BoxSurface(box);
		needUpdate=0;
	}

	friend TBox Union(const TBox &a,const TBox &b);
	typedef std::set<int>::iterator Iter;

	mutable Box box;
	mutable bool needUpdate;
	mutable float surface;

	std::set<int> inds;
	const TriVector *tris;
};

TBox Union(const TBox &a,const TBox &b) {
	TBox out(a.tris);
	std::set_union(a.inds.begin(),a.inds.end(),b.inds.begin(),b.inds.end(),
			std::insert_iterator<std::set<int> >(out.inds,out.inds.begin()) );
	return out;
}

struct WithinRange {
	WithinRange(float d) :dist(d) { }
	bool operator()(const TBox &b1,const TBox &b2) {
		return b1.GetBox().center.x+dist<b2.GetBox().center.x;
	}

	float dist;
};

vector<Box> GenBoxes(const TriVector &tris) {
	vector<TBox> boxes;

	Vec3f min(1.0f/0.0f,1.0f/0.0f,1.0f/0.0f),max(-1.0f/0.0f,-1.0f/0.0f,-1.0f/0.0f);

	for(int n=0;n<tris.size();n++) {
		min=VMin(VMin(min,tris[n].P1()),VMin(tris[n].P2(),tris[n].P3())); 
		max=VMax(VMax(max,tris[n].P1()),VMax(tris[n].P2(),tris[n].P3())); 

		TBox box(&tris); box.Insert(n);
		boxes.push_back(box);
	}
	Vec3f size=max-min;
	float maxDist=size.x*0.01f;

	while(boxes.size()>1) {
		printf("|"); fflush(stdout);
		bool noJoin=1;

		for(int n=0;n<boxes.size()-1;n++) {
		//	int first=std::lower_bound(&boxes[n+1],&boxes.back(),boxes[n],WithinRange(maxDist))-&boxes[0];
		//	int last=std::upper_bound(&boxes[n+1],&boxes.back(),boxes[n],WithinRange(maxDist))-&boxes[0];
			int first=n+1;
			int last=boxes.size()-1;

			for(int k=first;k<=last;k++) {
				TBox u=Union(boxes[n],boxes[k]);
				float t=boxes[n].Surface()+boxes[k].Surface()-u.Surface();
				t/=boxes[n].Surface()+boxes[k].Surface();

				if(t>0.f) {
					printf("."); fflush(stdout);
					boxes[n]=Union(boxes[n],boxes[k]);
					boxes[k--]=boxes.back(); last--;
					boxes.pop_back(); noJoin=0;
				}
				else if(t>-0.1f) for(int l=k+1;l<=last;l++) {
					TBox u2=Union(u,boxes[l]);
					float t2=boxes[n].Surface()+boxes[k].Surface()+boxes[l].Surface()-u2.Surface();
					t2/=boxes[n].Surface()+boxes[k].Surface()+boxes[l].Surface();
					if(t2>0.0f) {
						printf("."); fflush(stdout);
						boxes[n]=Union(u,boxes[k]);
						boxes[l]=boxes.back(); last--; boxes.pop_back();
						boxes[k--]=boxes.back(); last--; boxes.pop_back();
						noJoin=0;
						break;
					}
				}
			}
		}
		if(noJoin) break;
	}
	printf("\n");

	{
		vector<Box> out(boxes.size());
		for(int n=0;n<boxes.size();n++)
			out[n]=boxes[n].GetBox();
		return out;
	}
}

struct SortByX1 {
	bool operator()(const Triangle &a,const Triangle &b) { return a.P1().x<b.P1().x; }
};

int TreeVisMain(TriVector &tris) {
	for(int n=0;n<tris.size();n++) {
		Vec3f p1=tris[n].P1()*Vec3p(1.0f,-1.0f,1.0f);
		Vec3f p2=tris[n].P2()*Vec3p(1.0f,-1.0f,1.0f); 
		Vec3f p3=tris[n].P3()*Vec3p(1.0f,-1.0f,1.0f);
		tris[n]=Triangle(p2,p1,p3);
	}

	std::sort(tris.begin(),tris.end(),SortByX1());

	TriSurface(Vec3f(1,0,0),Vec3f(4,3,0),Vec3f(0,5,0));

	GLWindow window(resX,resY,0);

	int scene=glGenLists(1);
	glNewList(scene,GL_COMPILE);
	Render(tris);
	glEndList();

	{
		GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
		GLfloat mat_shininess[] = { 50.0 };
		GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
		glClearColor (0.0, 0.0, 0.0, 0.0);
		glShadeModel (GL_SMOOTH);

		glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glDepthFunc(GL_LEQUAL);
		glCullFace(GL_FRONT);
	}

	Camera cam;
	vector<char> selection(tris.size(),0);
	vector<Box> boxes;
	bool showScene=1;

	while(window.PollEvents()) {
		if(window.KeyDown(Key_esc)) break;
		Vec3f mPos=window.MousePos();
		cam.React(window);

		glViewport(0,0,resX,resY);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(90.0,4.0f/3.0f,10.0f,100000.0f);
		glMatrixMode(GL_MODELVIEW);

		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

		cam.Use();
		glEnable(GL_LIGHTING);
		if(showScene) glCallList(scene);
		glDisable(GL_LIGHTING);

		if(window.KeyDown('P')) showScene^=1;

		if(window.KeyDown('C'))
			for(int n=0;n<selection.size();n++)
				selection[n]=0;

		if(window.KeyDown('G')) boxes=GenBoxes(tris);

		if(window.MouseKey(0)||window.MouseKey(1)) {
			Vec3f orig[2],dir;
			orig[0]=Unproject(mPos.x,mPos.y,0.0f);
			orig[1]=Unproject(mPos.x,mPos.y,1.0f);
			dir=orig[1]-orig[0];
			float maxDist=Sqrt(dir|dir);
			dir/=maxDist;

			if(window.MouseKey(0)) for(int n=0;n<tris.size();n++) {
				float d[2];
				d[0]=tris[n].Collide(orig[0], dir);
				d[1]=tris[n].Collide(orig[1],-dir);
				if((d[0]>0.0f&&d[0]<maxDist)||(d[1]>0.0f&&d[1]<maxDist))
					selection[n]=1;
			}
			if(window.MouseKey(1)) for(int n=0;n<tris.size();n++) {
				float d[2];
				d[0]=tris[n].Collide(orig[0], dir);
				d[1]=tris[n].Collide(orig[1],-dir);
				if((d[0]>0.0f&&d[0]<maxDist)||(d[1]>0.0f&&d[1]<maxDist))
					selection[n]=0;
			}
		}

		for(int n=0;n<tris.size();n++)
			if(selection[n])
				RenderTri(tris[n],Vec3f(1,0,0));

		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	
			Box box;
			/*{ // compute axis aligned
				bool first=1;
				Vec3p min,max;
				for(int n=0;n<selection.size();n++) if(selection[n]) {
					const Triangle &tri=tris[n];
					if(first) { min=max=tri.P1(); first=0; }
					min=VMin(min,tri.P1());
					min=VMin(min,tri.P2());
					min=VMin(min,tri.P3());

					max=VMax(max,tri.P1());
					max=VMax(max,tri.P2());
					max=VMax(max,tri.P3());
				}
				box=Box(min,max);
			}*/
			{
				vector<Vec3f> points;
				for(int n=0;n<selection.size();n++) if(selection[n]) {
					const Triangle &tri=tris[n];
					points.push_back(tri.P1());
					points.push_back(tri.P2());
					points.push_back(tri.P3());
				}
				if(!points.size()) {
					points.push_back(Vec3f(0,0,0));
					points.push_back(Vec3f(1,1,1));
				}
				box=Box(&points[0],points.size());
			}

			if(window.Key('E')) {
				FILE *out=fopen("scenes/temp.obj","wb");
				Mat align((Vec4f)box.axis[0],(Vec4f)box.axis[1],(Vec4f)box.axis[2],Vec4f(0,0,0,1));

				int idx=0,count=0;
				Vec3f center(0,0,0);
				for(int n=0;n<tris.size();n++) {
					const Triangle &tri=tris[n];
					if(box.FullInside(tri)) {
						selection[n]=1;
						center+=tri.P1()+tri.P2()+tri.P3();
						count++;
					}
				}
				center/=float(count*3);

				for(int n=0;n<tris.size();n++) {
					const Triangle &tri=tris[n];
					if(selection[n]) {
						RenderTri(tris[n],Vec3f(0,0,1));

						Vec3f p1=(tri.P1()-center);
						Vec3f p2=(tri.P2()-center);
						Vec3f p3=(tri.P3()-center);
					//	p1=align*p1;
					//	p2=align*p2;
					//	p3=align*p3;
						fprintf(out,"v %f %f %f\n",p1.x,p1.y,p1.z);
						fprintf(out,"v %f %f %f\n",p2.x,p2.y,p2.z);
						fprintf(out,"v %f %f %f\n",p3.x,p3.y,p3.z);
						fprintf(out,"f %d %d %d\n",idx+2,idx+1,idx+3);
						idx+=3;
					}
				}
				fclose(out);
			}

			RenderBox(box,Vec4f(0,1,0,0.25f));

			for(int n=0;n<boxes.size();n++)
				RenderBox(boxes[n],Vec4f(n%3==0,n%3==1,n%3==2,0.25f));
			glDisable(GL_BLEND);
		}


		window.SwapBuffers();
	}
	return 0;
}


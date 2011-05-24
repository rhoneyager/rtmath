#include "StdAfx.h"
/*
#include "BitmapFont.h"

CBitmapFont::CBitmapFont(
    CDC* dc, 
    char* fontname)
{
    // Class constructor.
    // Stores the bitmaps for each character in its own display list
    // for later drawing via the wglUseFontBitmaps() call.

    if (dc && fontname && strlen(fontname) > 0) {

        m_pDC = dc;
        m_listbase = glGenLists(256);

        LOGFONT logfont;
        logfont.lfHeight = -12;
        logfont.lfWidth = 0;
        logfont.lfEscapement = 0;
        logfont.lfOrientation = logfont.lfEscapement;
        logfont.lfWeight = FW_NORMAL;
        logfont.lfItalic = FALSE;
        logfont.lfUnderline = FALSE;
        logfont.lfStrikeOut = FALSE;
        logfont.lfCharSet = ANSI_CHARSET;
        logfont.lfOutPrecision = OUT_DEFAULT_PRECIS;
        logfont.lfClipPrecision = CLIP_DEFAULT_PRECIS;
        logfont.lfQuality = DEFAULT_QUALITY;
        logfont.lfPitchAndFamily = FF_DONTCARE|DEFAULT_PITCH;
        lstrcpy(logfont.lfFaceName, fontname);

        CFont font;
        CFont* oldfont;
        BOOL success = font.CreateFontIndirect(&logfont);
        oldfont = m_pDC->SelectObject(&font);
        if (!success || 
            FALSE == wglUseFontBitmaps(m_pDC->m_hDC, 0, 256, m_listbase)) {
            glDeleteLists(m_listbase, 256);
            m_listbase = 0;
        }
        else {
            m_pDC->SelectObject(oldfont);
        }
    }
}

CBitmapFont::~CBitmapFont()
{
    // Class destructor.

    glDeleteLists(m_listbase, 256);
    m_listbase = 0;
}


void
CBitmapFont::DrawStringAt(
    GLfloat x, 
    GLfloat y, 
    GLfloat z, 
    char* s)
{
    // Draws the given text string at the given location.

    GLsizei len = GLsizei(strlen(s));
    if (s && len > 0) {
        glRasterPos3f(x, y, z);
		// Must save/restore the list base.
		glPushAttrib(GL_LIST_BIT);{
			glListBase(m_listbase);
			glCallLists(len, GL_UNSIGNED_BYTE, (const GLvoid*)s);
		} glPopAttrib();
    }
}


*/

/* From: http://www.opengl.org/resources/features/fontsurvey/
http://www.opengl.org/resources/features/fontsurvey/sooft/examples/WglFontDemo.zip
This demonstrates how to use both bitmap and outline fonts in an OpenGL program in Microsoft Windows. 
This program is based on the examples given in Ron Fosner's book, OpenGL Programming for Windows 95 
and Windows NT (aka, "The White Book"

    //CBitmapFont* m_bmf; // Bitmap font drawing.
    //COutlineFont* m_olf; // Outline font drawing.

    m_bmf = new CBitmapFont(m_pDC, "Courier");
    if (!m_bmf) {
        return FALSE;
    }

    m_olf = new COutlineFont(m_pDC, "Arial");
    if (!m_olf) {
        return FALSE;
    }



TODO: rewrite code
*/
#pragma once
/*
#ifndef _BitmapFont_h_
#define _BitmapFont_h_

//#include <GL/gl.h>
#include <string.h>

// CBitmapFont is a class which encapsulates the details necessary
// to draw bitmapped text in OpenGL on the Windows platform.

class CBitmapFont {
public:
    CBitmapFont(
        CDC* dc, 
        char* fontname);
    virtual ~CBitmapFont();
	
    void 
    DrawStringAt(
        float x, 
        float y, 
        float z, 
        char* s);

private:
    int m_listbase;
    CDC* m_pDC;

private:
    // Hide these.
    CBitmapFont() { }
    CBitmapFont(const CBitmapFont& obj) { }
    CBitmapFont& operator=(const CBitmapFont& obj) { return *this; }
};

#endif
*/
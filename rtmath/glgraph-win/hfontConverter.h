#pragma once
// Code taken from http://www.marshal-as.net/
// 8/26/2011 - HFONT and System::Drawing::Font
// From Nish.
//code starts here
namespace msclr
{
    namespace interop
    {
        template<> ref class context_node<System::Drawing::Font^, HFONT> : public context_node_base
        {
        private:
            System::Drawing::Font^ _font;
	
        public:
            context_node(System::Drawing::Font^% to, HFONT from)
            {
                to = _font = System::Drawing::Font::FromHfont((IntPtr)from);
            }
	
            ~context_node()
            {
                this->!context_node();
            }
	
        protected:
            !context_node()
            {
                delete _font;
            }
        };
	
        template<> ref class context_node<HFONT, System::Drawing::Font^> : public context_node_base
        {
        private:
            HFONT _hFont;
	
        public:
            context_node(HFONT& to, System::Drawing::Font^ from)
            {
                to = _hFont = (HFONT)from->ToHfont().ToPointer();
            }
	
            ~context_node()
            {
                this->!context_node();
            }
	
        protected:
            !context_node()
            {
                DeleteObject(_hFont);
            }
        };
    }
}
//code ends here

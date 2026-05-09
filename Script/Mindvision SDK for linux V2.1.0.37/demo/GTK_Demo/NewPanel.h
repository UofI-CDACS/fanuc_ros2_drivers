#ifndef NEWPANEL_H
#define NEWPANEL_H

//(*Headers(NewPanel)
#include <wx/panel.h>
//*)

class NewPanel: public wxPanel
{
	public:

		NewPanel(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~NewPanel();

		//(*Declarations(NewPanel)
		//*)

	protected:

		//(*Identifiers(NewPanel)
		//*)

	private:

		//(*Handlers(NewPanel)
		//*)

		DECLARE_EVENT_TABLE()
};

#endif

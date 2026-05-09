#include "NewPanel.h"

//(*InternalHeaders(NewPanel)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(NewPanel)
//*)

BEGIN_EVENT_TABLE(NewPanel,wxPanel)
	//(*EventTable(NewPanel)
	//*)
END_EVENT_TABLE()

NewPanel::NewPanel(wxWindow* parent,wxWindowID id)
{
	//(*Initialize(NewPanel)
	Create(parent, id, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("id"));
	//*)
}

NewPanel::~NewPanel()
{
	//(*Destroy(NewPanel)
	//*)
}


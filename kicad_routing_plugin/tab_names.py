"""The routing dialog's page names, in one place.

The dialog builds its notebooks from these, settings persistence saves and
restores the open page BY NAME through them, and the GUI gates assert the
order. A saved page used to be an index, which a page moved to the front
would silently re-point; LEGACY_* map such an index back to its page.
"""

#: The main notebook's pages, left to right.
AUTO_PAGE = "Auto"
MAIN_PAGES = (AUTO_PAGE, "Route", "Advanced", "Differential", "Fanout",
              "Planes", "Log", "About")

#: The Auto page's sub-tabs, left to right.
AUTO_SUBTABS = ("Autorouter", "AI Routing", "AI Placement")

#: The pages a saved `active_tab` / `ai_active_subtab` INDEX named before the
#: Auto page moved to the front (it was "AI Drive", index 5, with sub-tabs
#: Routing and Placement), as today's names.
LEGACY_MAIN_PAGES = ("Route", "Advanced", "Differential", "Fanout", "Planes",
                     AUTO_PAGE, "Log", "About")
LEGACY_AUTO_SUBTABS = ("AI Routing", "AI Placement")


def page_index(notebook, name):
    """The index of NOTEBOOK's page titled NAME, or None."""
    for i in range(notebook.GetPageCount()):
        if notebook.GetPageText(i) == name:
            return i
    return None

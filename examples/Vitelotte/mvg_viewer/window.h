/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#ifndef _MVG_VIEWER_WINDOW_
#define _MVG_VIEWER_WINDOW_


#include <QMainWindow>
#include <QAction>

#include "view.h"

class Window : public QMainWindow {
    Q_OBJECT

public:
    Window(QWidget* parent = NULL);

    View* view();

private slots:
    void setWireframe(QAction* act);

private:
    View* m_view;

    QAction* m_resetCamera;

    QActionGroup* m_wireframeGroup;
    QAction* m_setNoWireframe;
    QAction* m_setSimpleWireframe;
    QAction* m_setNodeWireframe;
    QAction* m_setOnlySimpleWireframe;
    QAction* m_setOnlyNodeWireframe;
};


#endif

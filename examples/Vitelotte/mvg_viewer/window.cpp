/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include "QToolBar"

#include "window.h"


Window::Window(QWidget* parent)
    : QMainWindow(parent),
      m_view(                  new View(this)),
      m_resetCamera(           new QAction("Reset camera", this)),
      m_wireframeGroup(        new QActionGroup(this)),
      m_setNoWireframe(        new QAction("No wireframe", this)),
      m_setSimpleWireframe(    new QAction("Simple wireframe", this)),
      m_setNodeWireframe(      new QAction("Node wireframe", this)),
      m_setOnlySimpleWireframe(new QAction("Only simple wireframe", this)),
      m_setOnlyNodeWireframe(  new QAction("Only node wireframe", this)) {
    setCentralWidget(m_view);

    m_resetCamera->setShortcut(QKeySequence("R"));
    connect(m_resetCamera, SIGNAL(triggered()),
            m_view, SLOT(resetTrackball()));

    m_setNoWireframe        ->setShortcut(QKeySequence("1"));
    m_setSimpleWireframe    ->setShortcut(QKeySequence("2"));
    m_setNodeWireframe      ->setShortcut(QKeySequence("3"));
    m_setOnlySimpleWireframe->setShortcut(QKeySequence("4"));
    m_setOnlyNodeWireframe  ->setShortcut(QKeySequence("5"));

    m_wireframeGroup->addAction(m_setNoWireframe);
    m_wireframeGroup->addAction(m_setSimpleWireframe);
    m_wireframeGroup->addAction(m_setNodeWireframe);
    m_wireframeGroup->addAction(m_setOnlySimpleWireframe);
    m_wireframeGroup->addAction(m_setOnlyNodeWireframe);
    connect(m_wireframeGroup, SIGNAL(triggered(QAction*)),
            this, SLOT(setWireframe(QAction*)));

    QList<QAction*> wireframeActions = m_wireframeGroup->actions();
    for(QList<QAction*>::iterator it = wireframeActions.begin();
        it != wireframeActions.end(); ++it) {
        (*it)->setCheckable(true);
        (*it)->setData(it - wireframeActions.begin());
    }

    m_setNoWireframe        ->setChecked(true);

    // Toolbar

    QToolBar* tb = addToolBar("Main toolbar");
    tb->addAction(m_resetCamera);
    tb->addSeparator();
    tb->addActions(m_wireframeGroup->actions());
}


View* Window::view() {
    return m_view;
}


void Window::setWireframe(QAction* act) {
    m_view->setWireMode(View::WireMode(act->data().toInt()));
}


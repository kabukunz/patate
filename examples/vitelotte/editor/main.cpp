#include "GL/glew.h"

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QGLFormat>
#include <QVBoxLayout>
#include <QSplitter>
#include <QMenuBar>
#include <QMenu>
#include <QAction>

#include "document.h"
#include "editor.h"
#include "valueEditor.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    QGLFormat glFormat;
    glFormat.setVersion(4, 0);
    QGLFormat::setDefaultFormat(glFormat);

    Editor* editor = new Editor;

    ValueEditor* valueEditor = new ValueEditor;

//    QWidget* editorsContainer = new QWidget;
//    editorsContainer->setLayout(new QVBoxLayout);
//    editorsContainer->layout()->setContentsMargins(QMargins());
//    editorsContainer->layout()->addWidget(valueEditor);
//    editorsContainer->layout()->addWidget(gradientEditor);

    QSplitter* content = new QSplitter;
    content->addWidget(editor);
    content->addWidget(valueEditor);
//    content->addWidget(editorsContainer);

    QMainWindow window;
    window.setCentralWidget(content);

    Document* doc = new Document(&window);
    doc->loadMesh("clean.mvg");
//    doc->loadMesh("test.mvg");
    editor->setDocument(doc);
    valueEditor->setDocument(doc);


    // File menu
    QAction* openAction = new QAction("Open...", &window);
    openAction->setShortcut(QKeySequence::Open);
    QObject::connect(openAction, SIGNAL(triggered()),
                     doc, SLOT(openLoadMeshDialog()));

    QAction* saveSourceAction = new QAction("Save source mesh as...", &window);
    saveSourceAction->setShortcut(QKeySequence::SaveAs);
    QObject::connect(saveSourceAction, SIGNAL(triggered()),
                     doc, SLOT(openSaveSourceMeshDialog()));

    QAction* saveFinalAction = new QAction("Save final mesh as...", &window);
    QObject::connect(saveFinalAction, SIGNAL(triggered()),
                     doc, SLOT(openSaveFinalMeshDialog()));

    QAction* exitAction = new QAction("Exit", &window);
    saveSourceAction->setShortcut(QKeySequence::Quit);
    QObject::connect(exitAction, SIGNAL(triggered()),
                     &window, SLOT(close()));

    QMenu* fileMenu = window.menuBar()->addMenu("File");
    fileMenu->addAction(openAction);
    fileMenu->addAction(saveSourceAction);
    fileMenu->addAction(saveFinalAction);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);


    // Edit Menu
    QAction* undoAction = doc->undoStack()->createUndoAction(&window);
    undoAction->setShortcut(QKeySequence::Undo);

    QAction* redoAction = doc->undoStack()->createRedoAction(&window);
    redoAction->setShortcut(QKeySequence::Redo);

    QMenu* editMenu = window.menuBar()->addMenu("Edit");
    editMenu->addAction(undoAction);
    editMenu->addAction(redoAction);


    // View Menu
    //QAction*


    window.resize(800, 600);
    window.show();

    return app.exec();
}

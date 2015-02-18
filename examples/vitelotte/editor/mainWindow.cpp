#include <QSplitter>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QPushButton>
#include <QColorDialog>

#include "document.h"
#include "editor.h"
#include "valueEditor.h"

#include "mainWindow.h"


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      m_document(0),
      m_editor(0),
      m_valueEditor(0),
      m_currentColor(Qt::black),
      m_splitter(0),
      m_fileMenu(0),
      m_editMenu(0),
      m_viewMenu(0),
      m_openAction(0),
      m_saveSourceAction(0),
      m_saveFinalAction(0),
      m_exitAction(0),
      m_undoAction(0),
      m_redoAction(0),
      m_editModeGroup(0),
      m_editCurvesAction(0),
      m_editNodesAction(0),
      m_wireframeAction(0),
      m_showMeshGroup(0),
      m_showBaseMeshAction(0),
      m_showFinalizedMeshAction(0),
      m_showSolvedMeshAction(0),
      m_colorButton(0),
      m_showMesh(Document::BASE_MESH),
      m_editMode(EDIT_NODES),
      m_interactionEnabled(false)
{
    m_splitter = new QSplitter;
    setCentralWidget(m_splitter);

    m_editor = new Editor;
    m_splitter->addWidget(m_editor);

    m_valueEditor = new ValueEditor;
    m_valueEditor->setEnabled(false);
    m_splitter->addWidget(m_valueEditor);

    m_document = new Document(this);
    m_editor->setDocument(m_document);
    m_valueEditor->setDocument(m_document);
    connect(m_document, SIGNAL(meshChanged()),
            this, SLOT(handleMeshChange()));

    connect(this, SIGNAL(currentColorChanged(QColor)),
            m_editor, SLOT(setPaintColor(QColor)));

    // File menu
    m_fileMenu = menuBar()->addMenu("File");

    m_openAction = new QAction("Open...", this);
    m_openAction->setShortcut(QKeySequence::Open);
    m_fileMenu->addAction(m_openAction);
    connect(m_openAction, SIGNAL(triggered()),
            m_document, SLOT(openLoadMeshDialog()));

    m_saveSourceAction = new QAction("Save source mesh as...", this);
    m_saveSourceAction->setShortcut(QKeySequence::SaveAs);
    m_fileMenu->addAction(m_saveSourceAction);
    connect(m_saveSourceAction, SIGNAL(triggered()),
            m_document, SLOT(openSaveSourceMeshDialog()));

    m_saveFinalAction = new QAction("Save final mesh as...", this);
    m_fileMenu->addAction(m_saveFinalAction);
    connect(m_saveFinalAction, SIGNAL(triggered()),
            m_document, SLOT(openSaveFinalMeshDialog()));

    m_fileMenu->addSeparator();

    m_exitAction = new QAction("Exit", this);
    m_saveSourceAction->setShortcut(QKeySequence::Quit);
    m_fileMenu->addAction(m_exitAction);
    connect(m_exitAction, SIGNAL(triggered()),
            this, SLOT(close()));


    // Edit Menu
    m_editMenu = menuBar()->addMenu("Edit");

    m_undoAction = m_document->undoStack()->createUndoAction(this);
    m_undoAction->setShortcut(QKeySequence::Undo);
    m_editMenu->addAction(m_undoAction);

    m_redoAction = m_document->undoStack()->createRedoAction(this);
    m_redoAction->setShortcut(QKeySequence::Redo);
    m_editMenu->addAction(m_redoAction);

    m_editMenu->addSeparator();

    m_editModeGroup = new QActionGroup(this);
    connect(m_editModeGroup, SIGNAL(triggered(QAction*)),
            this, SLOT(changeEditMode(QAction*)));

    m_editCurvesAction = new QAction("Edit curves", this);
    m_editCurvesAction->setCheckable(true);
    m_editCurvesAction->setChecked(true);
    m_editModeGroup->addAction(m_editCurvesAction);
    m_editMenu->addAction(m_editCurvesAction);

    m_editNodesAction = new QAction("Edit nodes", this);
    m_editNodesAction->setCheckable(true);
    m_editNodesAction->setChecked(false);
    m_editModeGroup->addAction(m_editNodesAction);
    m_editMenu->addAction(m_editNodesAction);


    // View Menu
    m_viewMenu = menuBar()->addMenu("View");

    m_showConstraintsAction = new QAction("Show constraints", this);
    m_showConstraintsAction->setCheckable(true);
    m_showConstraintsAction->setChecked(m_editor->showConstraints());
    m_viewMenu->addAction(m_showConstraintsAction);
    connect(m_showConstraintsAction, SIGNAL(triggered(bool)),
            m_editor, SLOT(setShowConstraints(bool)));

    m_wireframeAction = new QAction("Show wireframe", this);
    m_wireframeAction->setCheckable(true);
    m_wireframeAction->setChecked(m_editor->showWireframe());
    m_viewMenu->addAction(m_wireframeAction);
    connect(m_wireframeAction, SIGNAL(triggered(bool)),
            m_editor, SLOT(setShowWireframe(bool)));

    m_viewMenu->addSeparator();

    m_showMeshGroup = new QActionGroup(this);
    connect(m_showMeshGroup, SIGNAL(triggered(QAction*)),
            this, SLOT(changeShowMesh(QAction*)));

    m_showBaseMeshAction = new QAction("Show base mesh", this);
    m_showBaseMeshAction->setCheckable(true);
    m_showBaseMeshAction->setChecked(true);
    m_showMeshGroup->addAction(m_showBaseMeshAction);
    m_viewMenu->addAction(m_showBaseMeshAction);

    m_showFinalizedMeshAction = new QAction("Show finalized mesh", this);
    m_showFinalizedMeshAction->setCheckable(true);
    m_showFinalizedMeshAction->setChecked(false);
    m_showMeshGroup->addAction(m_showFinalizedMeshAction);
    m_viewMenu->addAction(m_showFinalizedMeshAction);

    m_showSolvedMeshAction = new QAction("Show solved mesh", this);
    m_showSolvedMeshAction->setCheckable(true);
    m_showSolvedMeshAction->setChecked(false);
    m_showMeshGroup->addAction(m_showSolvedMeshAction);
    m_viewMenu->addAction(m_showSolvedMeshAction);


    // Toolbar
    m_mainToolBar = addToolBar("main");

    m_colorButton = new QPushButton("Choose color");
    m_mainToolBar->addWidget(m_colorButton);
    connect(m_colorButton, SIGNAL(clicked()),
            this, SLOT(pickColor()));

    updateInteractionEnabled();
}


EditMode MainWindow::editMode() const
{
    return m_editMode;
}


void MainWindow::setEditMode(EditMode editMode)
{
    if(editMode != m_editMode)
    {
        m_editMode = editMode;
        m_editor->setEditMode(m_editMode);
        m_valueEditor->setVisible(m_editMode == EDIT_NODES);
        m_editCurvesAction->setChecked(m_editMode == EDIT_CURVES);
        m_editNodesAction->setChecked(m_editMode == EDIT_NODES);
        emit editModeChanged(m_editMode);
    }
}


void MainWindow::updateInteractionEnabled()
{
    bool enabled = m_document->mesh().nVertices() &&
                   m_editMode == EDIT_NODES &&
                   m_showMesh == Document::BASE_MESH;
    if(m_interactionEnabled != enabled)
    {
        m_valueEditor->setEnabled(enabled);
    }
    m_interactionEnabled = enabled;
}


void MainWindow::handleMeshChange()
{
    Mesh& mesh = m_document->mesh();
    setEditMode((mesh.nPointConstraints() || mesh.nCurves())?
                    EDIT_CURVES: EDIT_NODES);
    updateInteractionEnabled();
}


void MainWindow::changeShowMesh(QAction* action)
{
    if(action == m_showBaseMeshAction)
        m_showMesh = Document::BASE_MESH;
    else if(action == m_showFinalizedMeshAction)
        m_showMesh = Document::FINALIZED_MESH;
    else
    {
        assert(action == m_showSolvedMeshAction);
        m_showMesh = Document::SOLVED_MESH;
    }

    updateInteractionEnabled();
    m_editor->setShowMesh(m_showMesh);
    m_valueEditor->setShowMesh(m_showMesh);
}


void MainWindow::changeEditMode(QAction* action)
{
    if(action == m_editCurvesAction)
        setEditMode(EDIT_CURVES);
    else if(action == m_editNodesAction)
        setEditMode(EDIT_NODES);
}


void MainWindow::setCurrentColor(const QColor& color)
{
    if(m_currentColor != color)
    {
        m_currentColor = color;
        emit currentColorChanged(color);
    }
}


void MainWindow::pickColor()
{
    QColor color = QColorDialog::getColor(m_currentColor, this,
                                          "Choose pen color",
                                          QColorDialog::ShowAlphaChannel);
    if(color.isValid())
    {
        setCurrentColor(color);
    }
}

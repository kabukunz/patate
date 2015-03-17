#include <QSplitter>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QPushButton>
#include <QColorDialog>
#include <QIcon>

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
      m_setValueTypeGroup(0),
      m_setValueNoConitunous(0),
      m_setValueDiscontinuous(0),
      m_setDerivTypeGroup(0),
      m_setDerivContinuous(0),
      m_setDerivDiscontinuous(0),
      m_setDerivFlatLeft(0),
      m_setDerivFlatRight(0),
      m_setDerivFlat(0),
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
    connect(m_document, SIGNAL(meshUpdated()),
            this, SLOT(updateConstraintTypeActions()));
    connect(m_document, SIGNAL(selectionChanged()),
            this, SLOT(updateConstraintTypeActions()));

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

    m_editMenu->addSeparator();

    m_setValueTypeGroup = new QActionGroup(this);
    connect(m_setValueTypeGroup, SIGNAL(triggered(QAction*)),
            this, SLOT(setConstraintType(QAction*)));

    m_setDerivTypeGroup = new QActionGroup(this);
    connect(m_setDerivTypeGroup, SIGNAL(triggered(QAction*)),
            this, SLOT(setConstraintType(QAction*)));

    setupGroupAction(&m_setValueNoConitunous,         m_setValueTypeGroup,
                     m_editMenu, "Continuous value", ":/icons/consNoTear.png");
    setupGroupAction(&m_setValueDiscontinuous,           m_setValueTypeGroup,
                     m_editMenu, "Discontinuous value", ":/icons/consTear.png");

    m_editMenu->addSeparator();

    setupGroupAction(&m_setDerivContinuous,      m_setDerivTypeGroup,
                     m_editMenu, "Continuous gradient", ":/icons/consDerivNoTear.png");
    setupGroupAction(&m_setDerivDiscontinuous,        m_setDerivTypeGroup,
                     m_editMenu, "Discontinuous gradient", ":/icons/consDerivTear.png");
    setupGroupAction(&m_setDerivFlatLeft,    m_setDerivTypeGroup,
                     m_editMenu, "Flat left", ":/icons/consDerivFlatLeft.png");
    setupGroupAction(&m_setDerivFlatRight,   m_setDerivTypeGroup,
                     m_editMenu, "Flat right", ":/icons/consDerivFlatRight.png");
    setupGroupAction(&m_setDerivFlat,        m_setDerivTypeGroup,
                     m_editMenu, "Flat", ":/icons/consDerivFlat.png");

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

    m_mainToolBar->addSeparator();

    m_mainToolBar->addAction(m_setValueNoConitunous);
    m_mainToolBar->addAction(m_setValueDiscontinuous);

    m_mainToolBar->addSeparator();

    m_mainToolBar->addAction(m_setDerivContinuous);
    m_mainToolBar->addAction(m_setDerivDiscontinuous);
    m_mainToolBar->addAction(m_setDerivFlatLeft);
    m_mainToolBar->addAction(m_setDerivFlatRight);
    m_mainToolBar->addAction(m_setDerivFlat);

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
    updateConstraintTypeActions();
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


void MainWindow::updateConstraintTypeActions()
{
    Mesh& mesh = m_document->mesh();

    bool isPc = m_document->selection().isPointConstraint();
    m_setDerivDiscontinuous->setEnabled(!isPc);
    m_setDerivFlatLeft->setEnabled(!isPc);
    m_setDerivFlatRight->setEnabled(!isPc);
    if(isPc)
    {
        Mesh::PointConstraint pc = m_document->selection().pointConstraint();
        m_setValueTypeGroup->setEnabled(false);
        m_setDerivTypeGroup->setEnabled(true);
        if(mesh.isGradientConstraint(pc))
            m_setDerivFlat->setChecked(true);
        else
            m_setDerivContinuous->setChecked(true);
    }
    else if(m_document->selection().isCurve())
    {
        m_setValueTypeGroup->setEnabled(true);
        m_setDerivTypeGroup->setEnabled(true);

        Mesh::Curve c = m_document->selection().curve();
        if(mesh.valueTear(c))   m_setValueDiscontinuous->setChecked(true);
        else                    m_setValueNoConitunous->setChecked(true);

        bool gtear = mesh.gradientTear(c);
        bool cleft = !mesh.valueGradient(c, Mesh::GRADIENT_LEFT).empty();
        bool cright = !mesh.valueGradient(c, Mesh::GRADIENT_RIGHT).empty();
//        std::cout << "plop: " << ((gtear << 2) | (cright << 1) | cleft) << ": "
//                  << (gtear? "gDisc": "gCont") << ", "
//                  << (cleft? "consL": "freeL") << ", "
//                  << (cright? "consR": "freeR") << "\n";
        switch((gtear << 2) | (cright << 1) | cleft)
        {
        case 0: m_setDerivContinuous->setChecked(true); break;
        case 1: m_setDerivFlat->setChecked(true); break;
        case 2: m_setDerivContinuous->setChecked(true); break;
        case 3: m_setDerivFlat->setChecked(true); break;
        case 4: m_setDerivDiscontinuous->setChecked(true); break;
        case 5: m_setDerivFlatLeft->setChecked(true); break;
        case 6: m_setDerivFlatRight->setChecked(true); break;
        case 7: m_setDerivFlat->setChecked(true); break;
        }
    }
    else
    {
        m_setValueTypeGroup->setEnabled(false);
        m_setDerivTypeGroup->setEnabled(false);
    }
}


void MainWindow::setConstraintType(QAction* /*action*/)
{
    if(m_document->selection().isPointConstraint())
    {
        Mesh& mesh = m_document->mesh();
        Mesh::PointConstraint pc = m_document->selection().pointConstraint();
        if(m_setDerivContinuous->isChecked())
        {
            m_document->undoStack()->push(new SetPointConstraintGradient(
                                m_document, pc, mesh.unconstrainedGradientValue()));
        }
        else
        {
            m_document->undoStack()->push(new SetPointConstraintGradient(
                                m_document, pc, Mesh::Gradient::Zero(mesh.nCoeffs(), mesh.nDims())));
        }
        m_document->solve();
    }
    if(m_document->selection().isCurve())
    {
        Mesh::Curve c = m_document->selection().curve();
        Mesh& mesh = m_document->mesh();

        bool prevVTear = mesh.valueTear(c);
        bool prevGTear = mesh.gradientTear(c);
        bool prevCLeft = !mesh.valueGradient(c, Mesh::GRADIENT_LEFT).empty();
        bool prevCRight = !mesh.valueGradient(c, Mesh::GRADIENT_RIGHT).empty();

        bool nextVTear = m_setValueDiscontinuous->isChecked();
        bool nextGTear = !(m_setDerivContinuous->isChecked() || m_setDerivFlat->isChecked());
        bool nextCLeft = m_setDerivFlatLeft->isChecked() || m_setDerivFlat->isChecked();
        bool nextCRight = m_setDerivFlatRight->isChecked() || m_setDerivFlat->isChecked();

        bool doSetFlags = (prevVTear != nextVTear) || (prevGTear != nextGTear);
        bool doSetValueRight = prevVTear && !nextVTear;
        bool doSetDerivLeft = prevCLeft != nextCLeft;
        bool doSetDerivRight = prevCRight != nextCRight;
        unsigned nOp = doSetFlags + doSetValueRight + doSetDerivLeft + doSetDerivRight;

        if(nOp == 0) return;

        if(nOp != 1) m_document->undoStack()->beginMacro("Set curve type");

        // It is important to set gradients before flags, because Mesh::setFlags
        // modify gradients.
        if(doSetValueRight)
        {
    //        std::cout << "Set value right\n";
            m_document->undoStack()->push(new SetGradient(
                    m_document, c, Mesh::VALUE_RIGHT,
                    mesh.valueGradient(c, Mesh::GRADIENT_LEFT)));
        }

        Mesh::ValueGradient flatGradient;
        flatGradient[0] = Mesh::Value::Zero(mesh.nCoeffs());
        if(doSetDerivLeft)
        {
    //        std::cout << "Set deriv left\n";
            m_document->undoStack()->push(new SetGradient(
                    m_document, c, Mesh::GRADIENT_LEFT, nextCLeft?
                            flatGradient: Mesh::ValueGradient()));
        }
        if(doSetDerivRight)
        {
    //        std::cout << "Set deriv right\n";
            m_document->undoStack()->push(new SetGradient(
                    m_document, c, Mesh::GRADIENT_RIGHT, nextCRight?
                            flatGradient: Mesh::ValueGradient()));
        }

        if(doSetFlags)
        {
            unsigned flags = (nextVTear? Mesh::VALUE_TEAR: 0) | (nextGTear? Mesh::GRADIENT_TEAR: 0);
    //        std::cout << "Set flags: " << flags << "\n";
            m_document->undoStack()->push(new SetCurveFlags(m_document, c, flags));
        }

        if(nOp != 1) m_document->undoStack()->endMacro();

        m_document->solve();
    }
}


void MainWindow::setupGroupAction(QAction** act, QActionGroup* group,
                                  QMenu* menu, const char* name, const char* icon)
{
    (*act) = new QAction(name, this);
    (*act)->setCheckable(true);
    (*act)->setChecked(false);
    group->addAction(*act);
    menu->addAction(*act);

    if(icon)
    {
        QIcon iconObj(icon);
        assert(!iconObj.isNull());
        (*act)->setIcon(iconObj);
    }
}

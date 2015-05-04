#ifndef _MVG_EDITOR_MAIN_WINDOW_
#define _MVG_EDITOR_MAIN_WINDOW_


#include <QMainWindow>
#include <QColor>


#include "document.h"
#include "editor.h"


class QSplitter;
class QAction;
class QActionGroup;
class QMenu;
class QToolBar;
class QPushButton;

class ValueEditor;


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent=0);

    Document* document() { return m_document; }
    const Document* document() const { return m_document; }

    EditMode editMode() const;
    void setEditMode(EditMode editMode);

    bool isInteractionEnabled() const { return m_interactionEnabled; }

public slots:
    void updateInteractionEnabled();

protected slots:
    void handleMeshChange();
    void changeShowMesh(QAction* action);
    void changeEditMode(QAction* action);
    void setCurrentColor(const QColor& color);
    void pickColor();
    void updateConstraintTypeActions();
    void setConstraintType(QAction* action);

signals:
    void editModeChanged(int editMode);
    void currentColorChanged(const QColor& color);


private:
    void setupGroupAction(
            QAction** act, QActionGroup* group, QMenu* menu, const char* name,
            const char* icon = 0);

private:
    Document* m_document;
    Editor* m_editor;
    ValueEditor* m_valueEditor;

    QColor m_currentColor;

    QSplitter* m_splitter;

    QMenu* m_fileMenu;
    QMenu* m_editMenu;
    QMenu* m_viewMenu;

    QToolBar* m_mainToolBar;

    QAction* m_openAction;
    QAction* m_saveSourceAction;
    QAction* m_saveFinalAction;
    QAction* m_exitAction;

    QAction* m_undoAction;
    QAction* m_redoAction;
    QActionGroup* m_editModeGroup;
    QAction* m_editCurvesAction;
    QAction* m_editNodesAction;
    QActionGroup* m_setValueTypeGroup;
    QAction* m_setValueNoConitunous;
    QAction* m_setValueDiscontinuous;
    QActionGroup* m_setDerivTypeGroup;
    QAction* m_setDerivContinuous;
    QAction* m_setDerivDiscontinuous;
    QAction* m_setDerivFlatLeft;
    QAction* m_setDerivFlatRight;
    QAction* m_setDerivFlat;

    QAction* m_showConstraintsAction;
    QAction* m_wireframeAction;

    QActionGroup* m_showMeshGroup;
    QAction* m_showBaseMeshAction;
    QAction* m_showFinalizedMeshAction;
    QAction* m_showSolvedMeshAction;

    QPushButton* m_colorButton;

    Document::MeshType m_showMesh;
    EditMode m_editMode;
    bool m_interactionEnabled;
};


#endif

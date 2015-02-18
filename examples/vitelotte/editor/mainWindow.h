#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_


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

signals:
    void editModeChanged(int editMode);
    void currentColorChanged(const QColor& color);


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

#include "GL/glew.h"

#include <QApplication>
#include <QGLFormat>
#include <QWidget>
#include <QHBoxLayout>

#include "document.h"
#include "editor.h"
#include "valueEditor.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    QGLFormat glFormat;
    glFormat.setVersion(4, 0);
    QGLFormat::setDefaultFormat(glFormat);

    Document doc;
//    doc.loadMesh("clean.mvg");
    doc.loadMesh("test.mvg");

    Editor* editor = new Editor;
    editor->setDocument(&doc);

    ValueEditor* valueEditor = new ValueEditor;
    valueEditor->setDocument(&doc);

    QWidget window;
    window.setLayout(new QHBoxLayout);
    window.layout()->addWidget(editor);
    window.layout()->addWidget(valueEditor);
    window.resize(800, 600);
    window.show();

    return app.exec();
}

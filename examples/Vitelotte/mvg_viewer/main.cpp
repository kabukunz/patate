/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include "GL/glew.h"

#include <QApplication>
#include <QGLFormat>

#include "window.h"


int main(int argc, char** argv) {
    QApplication app(argc, argv);

    const char* filename = NULL;
    if(argc == 2) {
        filename = argv[1];
    } else if(argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [FILENAME]\n";
        return 1;
    }

    QGLFormat glFormat;
    // For some reasons, GLEW fails with a 4.0 context under Windows but need
    // it under Linux.
    // FIXME: clean this mess, and check under macOSX.
#ifndef _WIN32
    glFormat.setVersion(4, 0);
#endif
    glFormat.setSampleBuffers(true);
    glFormat.setSamples(16);
    QGLFormat::setDefaultFormat(glFormat);

    Window win;
    if(filename) {
        win.view()->openMvg(filename);
    }
    win.resize(800, 600);
    win.show();

    return app.exec();
}

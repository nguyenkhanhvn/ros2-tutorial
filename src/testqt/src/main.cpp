#include <QGuiApplication>
#include <QQmlEngine>
#include <QQmlFileSelector>
#include <QQuickView>

int main(int argc, char *argv[])
{
    QGuiApplication q_app (argc, argv);
    
    // Using QQuickView
    QQuickView view;
    view.setSource(QUrl("qrc:qml/Main.qml"));
    view.show();
    
    return q_app.exec ();
}
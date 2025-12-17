/**
 * @file main.cpp
 * @brief PumpBoard GUI Application Entry Point
 *
 * Qt-based GUI application for monitoring and controlling
 * the STM32F407xx PumpBoard hydraulic pump controller.
 */

#include <QApplication>
#include <QStyleFactory>
#include <QPalette>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Set application metadata
    app.setApplicationName("PumpBoard Controller GUI");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("PumpBoard");
    app.setOrganizationDomain("pumpboard.local");

    // Set dark theme
    app.setStyle(QStyleFactory::create("Fusion"));

    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(42, 42, 42));
    darkPalette.setColor(QPalette::AlternateBase, QColor(66, 66, 66));
    darkPalette.setColor(QPalette::ToolTipBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);

    // Disabled colors
    darkPalette.setColor(QPalette::Disabled, QPalette::WindowText, QColor(127, 127, 127));
    darkPalette.setColor(QPalette::Disabled, QPalette::Text, QColor(127, 127, 127));
    darkPalette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(127, 127, 127));

    app.setPalette(darkPalette);

    // Additional stylesheet for finer control
    app.setStyleSheet(
        "QToolTip { color: #ffffff; background-color: #2a2a2a; border: 1px solid #767676; }"
        "QGroupBox { font-weight: bold; border: 1px solid #767676; border-radius: 4px; margin-top: 8px; padding-top: 8px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
        "QTabWidget::pane { border: 1px solid #767676; }"
        "QTabBar::tab { background: #404040; border: 1px solid #767676; padding: 8px 16px; margin-right: 2px; }"
        "QTabBar::tab:selected { background: #505050; border-bottom-color: #505050; }"
        "QProgressBar { border: 1px solid #767676; border-radius: 3px; text-align: center; }"
        "QProgressBar::chunk { background-color: #2a82da; }"
        "QSlider::groove:horizontal { border: 1px solid #767676; height: 8px; background: #404040; border-radius: 4px; }"
        "QSlider::handle:horizontal { background: #2a82da; border: 1px solid #5c5c5c; width: 18px; margin: -5px 0; border-radius: 9px; }"
        "QSpinBox, QDoubleSpinBox { padding: 4px; }"
        "QPushButton { padding: 6px 12px; border-radius: 4px; }"
        "QPushButton:hover { background-color: #505050; }"
        "QPushButton:pressed { background-color: #404040; }"
    );

    // Create and show main window
    MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}

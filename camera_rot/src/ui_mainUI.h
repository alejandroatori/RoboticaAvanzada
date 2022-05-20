/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout;
    QSplitter *splitter;
    QFrame *frame;
    QFrame *control_frame;
    QVBoxLayout *verticalLayout_2;
    QLabel *top_camera_label;
    QHBoxLayout *horizontalLayout;
    QLabel *speed_label;
    QSpacerItem *horizontalSpacer_3;
    QLCDNumber *speed;
    QHBoxLayout *horizontalLayout_2;
    QLabel *posx_label;
    QSpacerItem *horizontalSpacer;
    QLCDNumber *pos_x;
    QHBoxLayout *horizontalLayout_3;
    QLabel *posz_label;
    QSpacerItem *horizontalSpacer_2;
    QLCDNumber *pos_y;
    QSpacerItem *verticalSpacer;
    QLabel *bottom_camera_label;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(1057, 698);
        verticalLayout = new QVBoxLayout(guiDlg);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        splitter = new QSplitter(guiDlg);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        frame = new QFrame(splitter);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(4);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setBaseSize(QSize(0, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        splitter->addWidget(frame);
        control_frame = new QFrame(splitter);
        control_frame->setObjectName(QString::fromUtf8("control_frame"));
        control_frame->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(control_frame->sizePolicy().hasHeightForWidth());
        control_frame->setSizePolicy(sizePolicy1);
        control_frame->setMinimumSize(QSize(0, 0));
        control_frame->setFrameShape(QFrame::StyledPanel);
        control_frame->setFrameShadow(QFrame::Raised);
        verticalLayout_2 = new QVBoxLayout(control_frame);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        top_camera_label = new QLabel(control_frame);
        top_camera_label->setObjectName(QString::fromUtf8("top_camera_label"));
        top_camera_label->setEnabled(true);
        top_camera_label->setMinimumSize(QSize(0, 200));

        verticalLayout_2->addWidget(top_camera_label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        speed_label = new QLabel(control_frame);
        speed_label->setObjectName(QString::fromUtf8("speed_label"));

        horizontalLayout->addWidget(speed_label);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        speed = new QLCDNumber(control_frame);
        speed->setObjectName(QString::fromUtf8("speed"));
        QFont font;
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        speed->setFont(font);
        speed->setSmallDecimalPoint(false);
        speed->setMode(QLCDNumber::Dec);
        speed->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout->addWidget(speed);


        verticalLayout_2->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        posx_label = new QLabel(control_frame);
        posx_label->setObjectName(QString::fromUtf8("posx_label"));

        horizontalLayout_2->addWidget(posx_label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        pos_x = new QLCDNumber(control_frame);
        pos_x->setObjectName(QString::fromUtf8("pos_x"));
        pos_x->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_2->addWidget(pos_x);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        posz_label = new QLabel(control_frame);
        posz_label->setObjectName(QString::fromUtf8("posz_label"));

        horizontalLayout_3->addWidget(posz_label);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        pos_y = new QLCDNumber(control_frame);
        pos_y->setObjectName(QString::fromUtf8("pos_y"));
        pos_y->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_3->addWidget(pos_y);


        verticalLayout_2->addLayout(horizontalLayout_3);

        verticalSpacer = new QSpacerItem(20, 240, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        bottom_camera_label = new QLabel(control_frame);
        bottom_camera_label->setObjectName(QString::fromUtf8("bottom_camera_label"));
        bottom_camera_label->setMinimumSize(QSize(0, 200));

        verticalLayout_2->addWidget(bottom_camera_label);

        splitter->addWidget(control_frame);

        verticalLayout->addWidget(splitter);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "giraff_viewer", nullptr));
        top_camera_label->setText(QApplication::translate("guiDlg", "CameraRGB", nullptr));
        speed_label->setText(QApplication::translate("guiDlg", "Speed", nullptr));
        posx_label->setText(QApplication::translate("guiDlg", "Position X", nullptr));
        posz_label->setText(QApplication::translate("guiDlg", "Position Y", nullptr));
        bottom_camera_label->setText(QApplication::translate("guiDlg", "CameraRGBD", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include <string>
#include <QDir>
#include <QSettings>
#include <QMouseEvent>
#include <QTextStream>
#include <QPainter>
#include <QCursor>
#include <QColor>
#include <QGlobalStatic>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QMessageBox>
#include <QAbstractItemModel>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>
#include <boost/chrono.hpp>
#include <filesystem>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "../algorithms/calibexecutive.h"

using namespace boost;
// namespace fs = std::filesystem;

#ifdef WIN32
static const char __CONFIGURATION_FOLDER[] = "c:/gaugecam/config/";
static const char __SETTINGS_FILEPATH[] = "c:/gaugecam/config/settingsWin.cfg";
#else
static const char __CONFIGURATION_FOLDER[] = "./config/";
static const char __SETTINGS_FILEPATH[] = "./config/settings.cfg";
#endif

static double Distance( const double x1, const double y1, const double x2, const double y2 )
{
    return sqrt( ( x2 - x1 ) * ( x2 - x1 ) + ( y2 - y1 ) * ( y2 - y1 ) );
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow( parent ),
    ui( new Ui::MainWindow ),
    m_folderLoadImages( "." ),
    m_folderSaveImages( "." ),
    m_imgWidth( MAX_IMAGE_SIZE.width ),
    m_imgHeight( MAX_IMAGE_SIZE.height ),
    m_bCaptured( false ),
    m_nCapturePos( 0 ),
    m_pComboBoxImageToView( nullptr ),
    m_pQImg( nullptr ),
    m_pLabelImgDisplay( nullptr ),
    m_scaleFactor( 1.0 ),
    m_lineOne( QPoint( 10, 130 ), QPoint( 130, 10 ) ),
    m_pRubberBand( nullptr ),
    m_rectROI( QRect( 0, 0, MAX_IMAGE_SIZE.width, MAX_IMAGE_SIZE.height ) ),
    m_rectRubberBand( QRect( 0, 0, MAX_IMAGE_SIZE.width, MAX_IMAGE_SIZE.height ) ),
    m_lineSearchPoly( LineSearchPoly( QPoint( 50, 50 ), QPoint( 100, 50 ),
                                       QPoint( 100, 100 ),  QPoint( 50, 100 ) ) )
{
    qRegisterMetaType< std::string >();

    ui->setupUi( this );

    QFile file( ":/docs/release_notes.html" );
    file.open( QFile::ReadOnly | QFile::Text );
    QTextStream stream( &file );
    if ( file.isOpen() )
    {
        ui->textBrowser_releaseNotes->setHtml( stream.readAll() );
        file.close();
    }
    else
    {
        ui->textBrowser_releaseNotes->setText( "Could not open release notes" );
    }

    file.setFileName( ":/LICENSE" );
    file.open( QFile::ReadOnly | QFile::Text );
    QTextStream licenseStream( &file );
    if ( file.isOpen() )
    {
        ui->textBrowser_license->setText( licenseStream.readAll() );
    }
    else
    {
        ui->textBrowser_license->setText( "Could not open LICENSE file" );
    }

    QWidget *spacerWidget = new QWidget( this );
    spacerWidget->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
    spacerWidget->setVisible( true );
    ui->mainToolBar->insertWidget( ui->actionExit, spacerWidget );

    QLabel *labelImageToView = new QLabel();
    labelImageToView->setText( QString::fromUtf8( "Image to view") );
    labelImageToView->setAlignment( Qt::AlignHCenter | Qt::AlignVCenter );

    m_pComboBoxImageToView = new QComboBox();
    m_pComboBoxImageToView->addItem( "Color" );
    m_pComboBoxImageToView->addItem( "Grayscale" );
    m_pComboBoxImageToView->addItem( "Overlay" );

    QWidget *widgetImageToView = new QWidget( this );

    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->addWidget( labelImageToView );
    vLayout->addWidget( m_pComboBoxImageToView );
    widgetImageToView->setLayout( vLayout );
    ui->mainToolBar->insertWidget( ui->actionZoomToFit, widgetImageToView );

    QWidget *spacerWidgetZoom = new QWidget( this );
    spacerWidgetZoom->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Preferred );
    spacerWidgetZoom->setVisible( true );
    ui->mainToolBar->insertWidget( ui->actionZoomToFit, spacerWidgetZoom );

    m_pLabelImgDisplay = new QLabel( ui->scrollArea_ImgDisplay );
    m_pLabelImgDisplay->setBackgroundRole( QPalette::Base );
    m_pLabelImgDisplay->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
    m_pLabelImgDisplay->setScaledContents( true );

    m_pLabelImgDisplay->resize( MAX_IMAGE_SIZE.width, MAX_IMAGE_SIZE.height );

    ui->scrollArea_ImgDisplay->setBackgroundRole( QPalette::Dark );
    ui->scrollArea_ImgDisplay->setWidgetResizable( false );
    ui->scrollArea_ImgDisplay->setWidget( m_pLabelImgDisplay );

    int ret = ReadSettings( __SETTINGS_FILEPATH );
    if ( 0 != ret )
    {
        QMessageBox::warning( this, "Read settings warning", "FAIL:  No settings found, using defaults" );
    }

    cv::Size sizeImg = cv::Size( m_imgWidth, m_imgHeight );
    ret = m_visApp.Init( __CONFIGURATION_FOLDER, sizeImg );
    ui->textEdit_msgs->append( 0 == ret ? "Settings load succeeded" : "Settings load failed" );

    ui->actionSaveVideo->setEnabled( false );

    createActions();
    createConnections();

    QPalette pal = ui->progressBar_imageLoad->palette();
    pal.setColor( QPalette::Normal, QPalette::Base, QColor( "green" ) );
    ui->progressBar_imageLoad->setPalette( pal );

    on_lineEdit_imageFolder_textEdited( ui->lineEdit_imageFolder->text() );
    on_actionZoom100_triggered();
    ui->widget_overlayCheckboxes->hide();

    GC_STATUS retVal = m_visApp.LoadCalib( ui->lineEdit_calibVisionResult_json->text().toStdString() );
    if ( GC_OK == retVal )
    {
        m_rectRubberBand.setLeft( qRound( static_cast< double >( m_rectROI.left() ) * m_scaleFactor ) );
        m_rectRubberBand.setTop( qRound( static_cast< double >( m_rectROI.top() ) * m_scaleFactor ) );
        m_rectRubberBand.setRight( qRound( static_cast< double >( m_rectROI.right() ) * m_scaleFactor ) );
        m_rectRubberBand.setBottom( qRound( static_cast< double >( m_rectROI.bottom() ) * m_scaleFactor ) );
        m_pRubberBand->setGeometry( m_rectRubberBand );
    }
    else
    {
        ui->textEdit_msgs->append( "Could not load calibration from " + ui->lineEdit_calibVisionResult_json->text() );
    }

    ui->checkBox_showCalib->setChecked( true );
    ui->checkBox_showSearchROI->setChecked( true );
    ui->checkBox_calibSearchROI->setChecked( true );
    ui->checkBox_showTargetROI->setChecked( true );
    ui->checkBox_showMoveFind->setChecked( true );
    m_pComboBoxImageToView->setCurrentText( "Overlay" );

    UpdateGUIEnables();
    UpdateCalibType();
    UpdateCalibSearchRegion();
}
MainWindow::~MainWindow()
{
    int ret = m_visApp.WriteSettings();
    if ( 0 != ret )
        QMessageBox::warning( this, "App write settings warning",
                              "FAIL:  Could not write application settings properly on program exit" );

    ret = WriteSettings( QString( __SETTINGS_FILEPATH ) );
    if ( 0 != ret )
        QMessageBox::warning( this, "GUI write settings warning",
                              "FAIL:  Could not write GUI settings properly on program exit" );

    delete m_pComboBoxImageToView;
    delete m_pRubberBand;
    delete m_pQImg;
    delete m_pLabelImgDisplay;
    delete ui;
}
void MainWindow::on_actionExit_triggered()
{
    if ( m_visApp.isRunningFindLine() )
    {
        GC_STATUS retVal = m_visApp.CalcLinesThreadFinish();
        QString msg = QString( "Stop running find line thread: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" );
        ui->statusBar->showMessage( msg );
    }
    if ( m_visApp.isRunningCreateGIF() )
    {
        GC_STATUS retVal = m_visApp.CreateGIFThreadFinish();
        QString msg = QString( "Stop running create GIF thread: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" );
        ui->statusBar->showMessage( msg );
    }
    close();
}
#ifndef QT_NO_CONTEXTMENU
void MainWindow::contextMenuEvent( QContextMenuEvent * )
{
}
#endif // QT_NO_CONTEXTMENU

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// helper methods
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::createActions()
{
//    actionRoiAdd = new QAction( "Add to list" );
//    actionRoiAdd->setStatusTip( "Add this ROI to the list" );
//    connect( actionRoiAdd, &QAction::)
}
void MainWindow::createConnections()
{
    connect( m_pComboBoxImageToView,  &QComboBox::currentTextChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showCalib, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->radioButton_calibDisplayScale, &QRadioButton::clicked, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->radioButton_calibDisplayGrid, &QRadioButton::clicked, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showFindLine, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showRowSums, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showDerivOne, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showDerivTwo, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showRANSAC, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showMoveFind, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showSearchROI, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showTargetROI, &QCheckBox::checkStateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_createFindLine_csvResultsFile, &QCheckBox::checkStateChanged, this, &MainWindow::UpdateGUIEnables );
    connect( ui->checkBox_createFindLine_annotatedResults, &QCheckBox::checkStateChanged, this, &MainWindow::UpdateGUIEnables );
    connect( ui->checkBox_saveSearchROI, &QCheckBox::checkStateChanged, this, &MainWindow::UpdateGUIEnables );
    connect( ui->checkBox_calibSearchROI, &QRadioButton::toggled, this, &MainWindow::UpdateCalibSearchRegion );
    connect( ui->actionToggleControls, &QAction::toggled, this, &MainWindow::UpdateGUIEnables );

    connect( this, SIGNAL( sig_visAppMessage(QString) ), this, SLOT( do_visAppMessage(QString) ) );
    connect( this, SIGNAL( sig_updateProgess(int) ),     this, SLOT( do_updateProgress(int) ) );
    connect( this, SIGNAL( sig_updateImage() ), this, SLOT( do_updateImage() ) );

    m_visApp.sigMessage.connect( bind( &MainWindow::on_visAppMessage, this, boost::placeholders::_1 ) );
    m_visApp.sigProgress.connect( bind( &MainWindow::on_updateProgress, this, boost::placeholders::_1 ) );
    m_visApp.sigTableAddRow.connect( bind( &MainWindow::on_tableAddRow, this, boost::placeholders::_1 ) );
    m_visApp.sigImageUpdate.connect( bind( &MainWindow::on_updateImage, this ) );

    // feature recipe connections

}
int MainWindow::ResizeImage( const int width, const int height )
{
    int ret = -1;
    if ( m_imgWidth == width && m_imgHeight == height && nullptr != m_pQImg )
    {
        return 0;
    }
    else
    {
        m_imgWidth = width;
        m_imgHeight = height;
        if ( nullptr != m_pRubberBand )
        {
            delete m_pRubberBand;
            m_pRubberBand = nullptr;
        }
        if ( nullptr != m_pQImg )
        {
            delete m_pQImg;
            m_pQImg = nullptr;
        }
        m_pLabelImgDisplay->resize( m_imgWidth, m_imgHeight );

        m_pQImg = new QImage( QSize( m_imgWidth, m_imgHeight ), QImage::Format_RGB32 );
        if ( nullptr == m_pQImg )
        {
            QMessageBox::warning( this, "Startup error",
                                  "Could not instantiate QBitmap for loading images" );
        }
        else
        {
            m_pQImg->fill( Qt::black );

            m_pRubberBand = new QRubberBand( QRubberBand::Rectangle, m_pLabelImgDisplay );
            if ( nullptr == m_pRubberBand )
            {
                ui->statusBar->showMessage( "FAIL:  Could not instantiate rubberband selector" );
            }
            else
            {
                m_pRubberBand->setGeometry( m_rectRubberBand );
                ret = 0;
            }
        }
    }

    return ret;
}
int MainWindow::ReadSettings( const QString filepath )
{
    int ret = 0;
    QSettings *pSettings = nullptr;
    if ( filepath.isEmpty() )
    {
        pSettings = new QSettings( "thrive", "VisTestApp" );
        if ( nullptr == pSettings )
            QMessageBox::warning( this, "Read settings warning",
                                  "FAIL:  Could not instantiate read settings object, using defaults" );
    }
    else
    {
        if ( !fs::exists( fs::path( filepath.toStdString() ).parent_path().string() ) )
        {
            bool bRet = fs::create_directories( fs::path( filepath.toStdString() ).parent_path() );
            if ( !bRet )
            {
                QString strMsg = QString( "FAIL:  Could not create folder for settings read file " ) + filepath;
                QMessageBox::warning( this, "Read settings warning", strMsg );
                ret = -1;
            }
        }
        if ( 0 == ret )
        {
            pSettings = new QSettings( filepath, QSettings::IniFormat );
            if ( nullptr == pSettings )
            {
                QString strMsg = QString( "FAIL:  Could not open settings read file " ) + filepath;
                QMessageBox::warning( this, "Read settings warning", strMsg );
                ret = -1;
            }
        }
    }
    if ( 0 == ret )
    {
        pSettings->beginGroup( "Image and ROI" );
        ResizeImage( m_imgWidth, m_imgHeight );
        m_rectROI.setLeft( pSettings->value( "roiLeft", 10 ).toInt() );
        m_rectROI.setTop( pSettings->value( "roiTop", 10 ).toInt() );
        m_rectROI.setWidth( pSettings->value( "roiWidth", 100 ).toInt() );
        m_rectROI.setHeight( pSettings->value( "roiHeight", 100 ).toInt() );

        int x1 = pSettings->value( "lineOneX1", 10 ).toInt();
        int y1 = pSettings->value( "lineOneY1", 10 ).toInt();
        int x2 = pSettings->value( "lineOneX2", 120 ).toInt();
        int y2 = pSettings->value( "lineOneY2", 120 ).toInt();
        m_lineOne = QLine( QPoint( x1, y1 ), QPoint( x2, y2 ) );

        m_lineSearchPoly.lftTop.setX( pSettings->value( "polyLftTopX", 10 ).toInt() );
        m_lineSearchPoly.lftTop.setY( pSettings->value( "polyLftTopY", 10 ).toInt() );
        m_lineSearchPoly.rgtTop.setX( pSettings->value( "polyRgtTopX", 120 ).toInt() );
        m_lineSearchPoly.rgtTop.setY( pSettings->value( "polyRgtTopY", 10 ).toInt() );
        m_lineSearchPoly.lftBot.setX( pSettings->value( "polyLftBotX", 120 ).toInt() );
        m_lineSearchPoly.lftBot.setY( pSettings->value( "polyLftBotY", 120 ).toInt() );
        m_lineSearchPoly.rgtBot.setX( pSettings->value( "polyRgtBotX", 10 ).toInt() );
        m_lineSearchPoly.rgtBot.setY( pSettings->value( "polyRgtBotY", 120 ).toInt() );

        m_folderLoadImages = pSettings->value( "loadFolder", "." ).toString();
        m_folderSaveImages = pSettings->value( "saveFolder", "." ).toString();

        ui->lineEdit_imageFolder->setText( pSettings->value( "imageFolder", m_folderLoadImages ).toString() );
        pSettings->endGroup();

        // vision stuff
        pSettings->beginGroup( "Vision" );
        ui->lineEdit_calibVisionResult_json->setText( pSettings->value( "calibJsonFileOut", QString( __CONFIGURATION_FOLDER ) + "calib.json" ).toString() );
        ui->checkBox_calibSearchROI->setChecked( !pSettings->value( "useWholeImage", true ).toBool() );
        ui->doubleSpinBox_octagonFacetLength->setValue( pSettings->value( "octagonFacetLength", 0.599 ).toDouble() ); // 7.1875 inches
        ui->doubleSpinBox_octagonZeroOffset->setValue( pSettings->value( "octagonZeroOffset", 2.0 ).toDouble() );
        ui->spinBox_moveSearchROIGrowPercent->setValue( pSettings->value( "moveSearchROIGrowPercent", 0 ).toInt() );

        ui->lineEdit_findLineTopFolder->setText( pSettings->value( "findLineFolder", QString( __CONFIGURATION_FOLDER ) ).toString() );
        ui->lineEdit_findLine_resultCSVFile->setText( pSettings->value( "findLineCSVOutPath", QString( __CONFIGURATION_FOLDER ) + "waterlevel.csv" ).toString() );

        bool isFolderOfImages = pSettings->value( "folderOfImages", true ).toBool();
        ui->radioButton_folderOfImages->setChecked( isFolderOfImages );
        ui->radioButton_folderOfFolders->setChecked( !isFolderOfImages );

        bool createCSV = pSettings->value( "createCSVCheckbox", true ).toBool();
        ui->checkBox_createFindLine_csvResultsFile->setChecked( createCSV );
        ui->lineEdit_findLine_annotatedResultFolder->setText( pSettings->value( "findLineAnnotatedOutFolder", QString( __CONFIGURATION_FOLDER ) ).toString() );
        bool enableSaveSearchROI = pSettings->value( "enableSaveSearchROI", true ).toBool();
        ui->groupBox_enableSaveSearchROI->setChecked( enableSaveSearchROI );
        ui->lineEdit_saveSearchROIFolder->setText( pSettings->value( "saveSearchROIFolder", QString( __CONFIGURATION_FOLDER ) ).toString() );
        ui->checkBox_createFindLine_annotatedResults->setChecked( pSettings->value( "createAnnotationCheckbox", false ).toBool() );
        ui->lineEdit_saveSearchROIFolder->setText( pSettings->value( "saveSearchROIFolder", QString( __CONFIGURATION_FOLDER ) ).toString() );
        ui->checkBox_saveSearchROI->setChecked( pSettings->value( "saveSearchROICheckbox", false ).toBool() );

        ui->spinBox_timeStringPosZero->setValue( pSettings->value( "timestampStringStartPos", 10 ).toInt() );
        ui->radioButton_dateTimeInFilename->setChecked( pSettings->value( "timestampFromFilename", true ).toBool() );
        ui->radioButton_dateTimeInEXIF->setChecked( pSettings->value( "timestampFromEXIF", false ).toBool() );
        ui->lineEdit_timestampFormat->setText( pSettings->value( "timestampFormat", "yy-mm-ddTHH-MM" ).toString() );
        pSettings->endGroup();

        delete pSettings;
        pSettings = nullptr;
    }

    return ret;
}
int MainWindow::WriteSettings( const QString filepath )
{
    int ret = 0;
    QSettings *pSettings = nullptr;
    if ( filepath.isEmpty() )
    {
        pSettings = new QSettings( "thrive", "VisTestApp" );
        if ( nullptr == pSettings )
        {
            QMessageBox::warning( this, "Write settings warning",
                                  "FAIL:  Could not instantiate read settings object, using defaults" );
            ret = -1;
        }
    }
    else
    {
        pSettings = new QSettings( filepath, QSettings::IniFormat );
        if ( nullptr == pSettings )
        {
            QString strMsg;
            QMessageBox::warning( this, "Write settings warning",
                                 QString( "FAIL:  Could not open settings read file " ) + filepath );
            ret = -1;

        }
    }

    if ( 0 == ret )
    {
        // image stuff
        pSettings->beginGroup( "Image and ROI" );
        pSettings->setValue( "width", m_imgWidth );
        pSettings->setValue( "height", m_imgHeight );
        pSettings->setValue( "roiLeft", m_rectROI.left() );
        pSettings->setValue( "roiTop", m_rectROI.top() );
        pSettings->setValue( "roiWidth", m_rectROI.width() );
        pSettings->setValue( "roiHeight", m_rectROI.height() );

        pSettings->setValue( "lineOneX1", m_lineOne.p1().x() );
        pSettings->setValue( "lineOneY1", m_lineOne.p1().y() );
        pSettings->setValue( "lineOneX2", m_lineOne.p2().x() );
        pSettings->setValue( "lineOneY2", m_lineOne.p2().y() );

        pSettings->setValue( "polyLftTopX", m_lineSearchPoly.lftTop.x() );
        pSettings->setValue( "polyLftTopY", m_lineSearchPoly.lftTop.y() );
        pSettings->setValue( "polyRgtTopX", m_lineSearchPoly.rgtTop.x() );
        pSettings->setValue( "polyRgtTopY", m_lineSearchPoly.rgtTop.y() );
        pSettings->setValue( "polyLftBotX", m_lineSearchPoly.lftBot.x() );
        pSettings->setValue( "polyLftBotY", m_lineSearchPoly.lftBot.y() );
        pSettings->setValue( "polyRgtBotX", m_lineSearchPoly.rgtBot.x() );
        pSettings->setValue( "polyRgtBotY", m_lineSearchPoly.rgtBot.y() );

        pSettings->setValue( "loadFolder", m_folderLoadImages );
        pSettings->setValue( "saveFolder", m_folderSaveImages );

        pSettings->setValue( "imageFolder", ui->lineEdit_imageFolder->text() );
        pSettings->endGroup();

        // vision stuff
        pSettings->beginGroup( "Vision" );
        pSettings->setValue( "calibJsonFileOut", ui->lineEdit_calibVisionResult_json->text() );
        pSettings->setValue( "useWholeImage", !ui->checkBox_calibSearchROI->isChecked() );
        pSettings->setValue( "octagonFacetLength", ui->doubleSpinBox_octagonFacetLength->value() );
        pSettings->setValue( "octagonZeroOffset", ui->doubleSpinBox_octagonZeroOffset->value() );
        pSettings->setValue( "moveSearchROIGrowPercent", ui->spinBox_moveSearchROIGrowPercent->value() );

        pSettings->setValue( "findLineFolder", ui->lineEdit_findLineTopFolder->text() );
        pSettings->setValue( "findLineCSVOutPath", ui->lineEdit_findLine_resultCSVFile->text() );
        pSettings->setValue( "folderOfImages", ui->radioButton_folderOfImages->isChecked() );
        pSettings->setValue( "createCSVCheckbox", ui->checkBox_createFindLine_csvResultsFile->isChecked() );
        pSettings->setValue( "findLineAnnotatedOutFolder", ui->lineEdit_findLine_annotatedResultFolder->text() );
        pSettings->setValue( "enableSaveSearchROI", ui->groupBox_enableSaveSearchROI->isChecked() );
        pSettings->setValue( "saveSearchROIFolder", ui->lineEdit_saveSearchROIFolder->text() );
        pSettings->setValue( "createAnnotationCheckbox", ui->checkBox_createFindLine_annotatedResults->isChecked() );
        pSettings->setValue( "saveSearchROIFolder", ui->lineEdit_saveSearchROIFolder->text() );
        pSettings->setValue( "saveSearchROICheckbox", ui->checkBox_saveSearchROI->isChecked() );

        pSettings->setValue( "timestampStringStartPos", ui->spinBox_timeStringPosZero->value() );
        pSettings->setValue( "timestampFromEXIF", ui->radioButton_dateTimeInEXIF->isChecked() );
        pSettings->setValue( "timestampFormat", ui->lineEdit_timestampFormat->text() );
        pSettings->endGroup();

        delete pSettings;
        pSettings = nullptr;
    }

    return ret;
}
void MainWindow::ZoomTo( const int width, const int height )
{
    double widthScrollArea = static_cast< double >( ui->scrollArea_ImgDisplay->width() );
    double heightScrollArea = static_cast< double >( ui->scrollArea_ImgDisplay->height() );
    double widthScale = widthScrollArea / static_cast< double >( width );
    double heightScale = heightScrollArea / static_cast< double >( height );
    int nScale = qRound( 99.5 * std::min( widthScale, heightScale ) );
    if ( nScale == ui->horizontalSlider_zoom->value() )
    {
        m_scaleFactor = static_cast< double >( ui->horizontalSlider_zoom->value() ) / 100.0;
        ScaleImage();
    }
    else
    {
        ui->horizontalSlider_zoom->setValue( nScale );
    }
}
void MainWindow::UpdateTargetRect()
{
    cv::Rect rect;
    GC_STATUS retVal = m_visApp.GetTargetSearchROI( rect );
    if ( GC_OK == retVal )
    {
        m_rectROI.setX( rect.x );
        m_rectROI.setY( rect.y );
        m_rectROI.setWidth( rect.width );
        m_rectROI.setHeight( rect.height );
        m_rectRubberBand.setLeft( qRound( static_cast< double >( m_rectROI.left() ) * m_scaleFactor ) );
        m_rectRubberBand.setTop( qRound( static_cast< double >( m_rectROI.top() ) * m_scaleFactor ) );
        m_rectRubberBand.setRight( qRound( static_cast< double >( m_rectROI.right() ) * m_scaleFactor ) );
        m_rectRubberBand.setBottom( qRound( static_cast< double >( m_rectROI.bottom() ) * m_scaleFactor ) );
        m_pRubberBand->setGeometry( m_rectRubberBand );
    }
    else
    {
        ui->statusBar->showMessage( "Could not set target search ROI properly" );
    }
}
void MainWindow::UpdateCalibSearchRegion()
{
    if ( ui->checkBox_calibSearchROI->isChecked() )
    {
        QString msg;
        ui->label_calibCurrentROI->setText( msg.asprintf( "x=%d  y=%d  w=%d  h=%d",
                                                          m_rectROI.x(), m_rectROI.y(),
                                                          m_rectROI.width(), m_rectROI.height() ) );
    }
    else
    {
        ui->label_calibCurrentROI->setText( "Whole image" );
    }
}
void MainWindow::UpdateCalibType()
{
    ui->groupBox_calibOctagon->setEnabled( true );
    ui->doubleSpinBox_octagonFacetLength->setEnabled( true );
    ui->label_moveSearchROI->setEnabled( false );
    ui->spinBox_moveSearchROIGrowPercent->setEnabled( false );
}
void MainWindow::UpdateGUIEnables()
{
    ui->lineEdit_findLine_resultCSVFile->setEnabled( ui->checkBox_createFindLine_csvResultsFile->isChecked() );
    ui->toolButton_findLine_resultCSVFile_browse->setEnabled( ui->checkBox_createFindLine_csvResultsFile->isChecked() );
    ui->lineEdit_findLine_annotatedResultFolder->setEnabled( ui->checkBox_createFindLine_annotatedResults->isChecked() );
    ui->toolButton_findLine_annotatedResultFolder_browse->setEnabled( ui->checkBox_createFindLine_annotatedResults->isChecked() );
    ui->lineEdit_saveSearchROIFolder->setEnabled( ui->checkBox_saveSearchROI->isChecked() );
    ui->toolButton_saveSearchROIFolder_browse->setEnabled( ui->checkBox_saveSearchROI->isChecked() );
    ui->widget_overlayCheckboxes->setHidden( !ui->actionToggleControls->isChecked() );
}
void MainWindow::UpdatePixmapTarget()
{
#ifdef _WIN32
    update();
#else
    UpdatePixmap();
#endif
}
int MainWindow::UpdatePixmap()
{
    int ret = 0;
    if ( nullptr == m_pQImg )
    {
        ui->statusBar->showMessage( "FAIL: Image display buffer not initialized" );
        ret = -1;
    }
    else
    {
        m_pQImg->fill( 0 );
        IMG_BUFFERS nColorType;
        if ( 0 == m_pComboBoxImageToView->currentText().compare( "Grayscale") )
            nColorType = BUF_GRAY;
        else if ( 0 == m_pComboBoxImageToView->currentText().compare( "Overlay") )
            nColorType = BUF_OVERLAY;
        else if ( 0 == m_pComboBoxImageToView->currentText().compare( "Color") )
            nColorType = BUF_RGB;
        else
        {
            ui->statusBar->showMessage( "Invalid color type selected for save" );
            ret = -1;
        }

        if ( 0 == ret )
        {
            int overlays = OVERLAYS_NONE;
            if ( ui->checkBox_showCalib->isChecked() )
            {
                overlays += ui->radioButton_calibDisplayScale->isChecked() ? CALIB_SCALE : CALIB_GRID;
            }
            overlays += ui->checkBox_showFindLine->isChecked() ? FINDLINE : OVERLAYS_NONE;
            overlays += ui->checkBox_showRowSums->isChecked() ? DIAG_ROWSUMS : OVERLAYS_NONE;
            overlays += ui->checkBox_showDerivOne->isChecked() ? FINDLINE_1ST_DERIV : OVERLAYS_NONE;
            overlays += ui->checkBox_showDerivTwo->isChecked() ? FINDLINE_2ND_DERIV : OVERLAYS_NONE;
            overlays += ui->checkBox_showRANSAC->isChecked() ? RANSAC_POINTS : OVERLAYS_NONE;
            overlays += ui->checkBox_showMoveFind->isChecked() ? MOVE_FIND : OVERLAYS_NONE;
            overlays += ui->checkBox_showSearchROI->isChecked() ? SEARCH_ROI : OVERLAYS_NONE;
            overlays += ui->checkBox_showTargetROI->isChecked() ? TARGET_ROI : OVERLAYS_NONE;

            gc::GC_STATUS retVal = m_visApp.GetImage( cv::Size( m_pQImg->width(), m_pQImg->height() ),
                                                      static_cast< size_t >( m_pQImg->bytesPerLine() ),
                                                      CV_8UC4, m_pQImg->scanLine( 0 ), nColorType,
                                                      static_cast< IMG_DISPLAY_OVERLAYS >( overlays ) );
            if ( GC_OK != retVal )
            {
                ui->statusBar->showMessage( QString( "Paint event failed with color " ) + QString::fromStdString( to_string( nColorType ) ) );
            }

            QPixmap pixmap = QPixmap::fromImage( *m_pQImg );
            if ( ui->actionSetRuler->isChecked() )
            {
                int lineWidth = qRound( 1.5 / m_scaleFactor );
                int endRadius = qRound( 7.0 / m_scaleFactor );
                QPainter painter( &pixmap );
                QPen pen1( Qt::SolidLine );
                pen1.setWidth( 3 );
                pen1.setColor( Qt::red );

                painter.setPen( pen1 );
                painter.drawEllipse( m_lineOne.p2(), endRadius, endRadius );

                pen1.setColor( Qt::green );
                painter.setPen( pen1 );
                painter.drawEllipse( m_lineOne.p1(), endRadius, endRadius );

                pen1.setWidth( lineWidth );
                pen1.setColor( Qt::yellow );
                painter.setPen( pen1 );
                painter.drawLine( m_lineOne );
            }
            else if ( ui->actionSetSearchPoly->isChecked() )
            {
                int lineWidth = qRound( 1.5 / m_scaleFactor );
                int endRadius = qRound( 7.0 / m_scaleFactor );
                QPainter painter( &pixmap );
                QPen pen1( Qt::SolidLine );

                pen1.setWidth( lineWidth );
                pen1.setColor( Qt::blue );
                painter.setPen( pen1 );

                painter.drawLine( QLine( m_lineSearchPoly.lftTop, m_lineSearchPoly.rgtTop ) );
                painter.drawLine( QLine( m_lineSearchPoly.rgtTop, m_lineSearchPoly.rgtBot ) );
                painter.drawLine( QLine( m_lineSearchPoly.rgtBot, m_lineSearchPoly.lftBot ) );
                painter.drawLine( QLine( m_lineSearchPoly.lftBot, m_lineSearchPoly.lftTop ) );

                pen1.setWidth( 3 );
                pen1.setColor( Qt::red );
                painter.setBrush( Qt::red );
                painter.setPen( pen1 );

                painter.drawEllipse( m_lineSearchPoly.lftTop, endRadius, endRadius );
                painter.drawEllipse( m_lineSearchPoly.rgtTop, endRadius, endRadius );
                painter.drawEllipse( m_lineSearchPoly.rgtBot, endRadius, endRadius );
                painter.drawEllipse( m_lineSearchPoly.lftBot, endRadius, endRadius );
            }
            m_pLabelImgDisplay->setPixmap( pixmap );
        }
    }
    return ret;
}
int MainWindow::ScaleImage()
{
    int ret = 0;
    if ( numeric_limits< double >::epsilon() > m_scaleFactor )
    {
        ui->statusBar->showMessage( "FAIL:  Invalid zoom factor" );
        ret = -1;
    }
    else
    {
        m_pLabelImgDisplay->resize( m_scaleFactor * QSize( m_imgWidth, m_imgHeight ) );
        m_pRubberBand->hide();
        if ( ui->actionSetROI->isChecked() )
        {
            m_rectRubberBand.setLeft( qRound( static_cast< double >( m_rectROI.left() ) * m_scaleFactor ) );
            m_rectRubberBand.setTop( qRound( static_cast< double >( m_rectROI.top() ) * m_scaleFactor ) );
            m_rectRubberBand.setRight( qRound( static_cast< double >( m_rectROI.right() ) * m_scaleFactor ) );
            m_rectRubberBand.setBottom( qRound( static_cast< double >( m_rectROI.bottom() ) * m_scaleFactor ) );
            m_pRubberBand->setGeometry( m_rectRubberBand );
            m_pRubberBand->show();
        }
        UpdatePixmapTarget();
    }
    return ret;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// event overloads
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::paintEvent( QPaintEvent * )
{
#ifdef _WIN32
    UpdatePixmap();
#endif
}
void MainWindow::on_pushButton_clearTable_clicked() { ClearTable(); }
void MainWindow::on_tableAddRow( const string row_string ) { AddRow( row_string ); }
void MainWindow::on_updateImage() { emit sig_updateImage(); }
void MainWindow::do_updateImage()
{
    UpdatePixmapTarget();
}
void MainWindow::on_updateProgress( const int value ) { emit sig_updateProgess( value ); }
void MainWindow::do_updateProgress( const int value ) { ui->progressBar_imageLoad->setValue( std::min( 100, std::max( 0, value ) ) ); }
void MainWindow::on_visAppMessage( string msg ) { emit sig_visAppMessage( QString::fromStdString( msg ) ); }
void MainWindow::do_visAppMessage( QString msg )
{
    if ( msg.contains( "update image only" ) )
    {
        UpdatePixmap();
    }
    else
    {
        if ( msg.contains( "Timestamp failure" ) )
        {
            on_pushButton_findLine_stopFolderProcess_clicked();
            ui->statusBar->showMessage( msg );
        }
        else if ( msg.contains( "Folder run complete" ) )
        {
            ui->pushButton_findLine_processFolder->setEnabled( true );
            ui->pushButton_findLine_stopFolderProcess->setEnabled( false );
            ui->pushButton_createAnimation->setEnabled( true );
            ui->pushButton_animationStop->setEnabled( false );
        }
        else if ( msg.contains( "Create GIF complete" ) )
        {
            ui->pushButton_createAnimation->setEnabled( true );
            ui->pushButton_animationStop->setEnabled( false );
            ui->pushButton_findLine_processFolder->setEnabled( true );
            ui->pushButton_findLine_stopFolderProcess->setEnabled( false );
        }
        ui->textEdit_msgs->setText( msg );
    }
}
void MainWindow::keyPressEvent( QKeyEvent *event )
{
    // Check if the Ctrl key is pressed
    if ( event->modifiers() & Qt::ControlModifier )
    {
        ui->scrollArea_ImgDisplay->verticalScrollBar()->setEnabled( false );
    }
}
void MainWindow::keyReleaseEvent( QKeyEvent * )
{
    // Check if the Ctrl key is pressed
    if ( !ui->scrollArea_ImgDisplay->verticalScrollBar()->isEnabled() )
    {
        ui->scrollArea_ImgDisplay->verticalScrollBar()->setEnabled( true );
    }
}
void MainWindow::wheelEvent( QWheelEvent *event )
{
    // Check if the Ctrl key is pressed
    if ( event->modifiers() & Qt::ControlModifier )
    {
        // Custom handling: Display the delta of the wheel event in the label
        int delta = event->angleDelta().y();
        if ( 0 < delta )
        {
            int value = ( ui->horizontalSlider_zoom->value() ) + 5;
            ui->horizontalSlider_zoom->setValue( 400 < value ? 400 : value );
        }
        else if ( 0 > delta )
        {
            int value = ( ui->horizontalSlider_zoom->value() ) - 5;
            ui->horizontalSlider_zoom->setValue( 1 > value ? 1 : value );
        }
        event->accept();  // Custom handling is done, stop further processing if desired
    }
    else
    {
        // No Ctrl pressed: Let normal processing occur by calling the base class implementation
        QWidget::wheelEvent(event);
    }
}
void MainWindow::mousePressEvent( QMouseEvent *pEvent )
{
    QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, pEvent->pos() );
    pt.setY( pt.y() - ui->mainToolBar->height() );
    int sensitivityRadius = qRound( 7.0 / m_scaleFactor );

    if ( nullptr != m_pRubberBand && ui->actionSetROI->isChecked() )
    {
        int ret = m_roiAdjust.EvalRectCapturePt( m_rectRubberBand, pt, sensitivityRadius, m_nCapturePos, m_ptCapture );
        m_bCaptured = ( ( 0 >= m_nCapturePos ) || ( 0 != ret ) ) ? false : true;
    }
    else if ( ui->actionSetSearchPoly->isChecked() )
    {
        int ret = m_roiAdjust.EvalPolyCapturePt( m_lineSearchPoly, pt, m_scaleFactor, sensitivityRadius, m_nCapturePos, m_ptCapture );
        m_bCaptured = ( ( 0 >= m_nCapturePos ) || ( 0 != ret ) ) ? false : true;
    }
    else if ( ui->actionSetRuler->isChecked() )
    {
        int ret = m_roiAdjust.EvalRulerCapturePt( m_lineOne, pt, m_scaleFactor, sensitivityRadius, m_nCapturePos, m_ptCapture );
        m_bCaptured = ( ( 0 >= m_nCapturePos ) || ( 0 != ret ) ) ? false : true;
    }
}
void MainWindow::mouseMoveEvent( QMouseEvent *pEvent )
{
    QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, pEvent->pos() );
    pt.setY( pt.y() - ui->mainToolBar->height() );
    int nX = qRound( static_cast< double >( pt.x() ) / m_scaleFactor );
    int nY = qRound( static_cast< double >( pt.y() ) / m_scaleFactor );
    if ( 0 <= nX && m_imgWidth > nX && 0 <= nY && m_imgHeight > nY )
    {
        if ( !ui->actionSetRuler->isChecked() && !ui->actionSetSearchPoly->isChecked() )
        {
            cv::Point2d world;
            GC_STATUS retVal = m_visApp.PixelToWorld( cv::Point2d( nX, nY ), world );
            if ( GC_OK != retVal )
            {
                world = cv::Point2d( -9999999.9, -9999999.9 );
            }
            QString strMsg = QString( "Pix X=" ) + QString::number( nX ) + " Y=" + QString::number( nY );
            ui->textEdit_measures->setText( strMsg );
            char buf[ 32 ];
            sprintf( buf, "Wor X1=%.1f Y1=%.1f", world.x, world.y );
            ui->textEdit_measures->append( buf );
        }
    }
    else
    {
        ui->textEdit_msgs->setText( "Off image" );
    }

    if ( m_bCaptured )
    {
        if ( nullptr != m_pRubberBand && ui->actionSetROI->isChecked() )
        {
            int ret = m_roiAdjust.TestAgainstRubberBands( pt, m_pLabelImgDisplay->size(), m_rectRubberBand,
                                                          m_rectROI, m_nCapturePos, m_scaleFactor, m_ptCapture );
            if ( 0 == ret )
            {
                m_pRubberBand->setGeometry( m_rectRubberBand );
            }
        }
        else if ( ui->actionSetSearchPoly->isChecked() )
        {
            int ret = m_roiAdjust.TestAgainstPoly( pt, m_pLabelImgDisplay->size(),
                                                   m_lineSearchPoly, m_nCapturePos, m_scaleFactor, m_ptCapture );
            if ( 0 == ret )
            {
                UpdatePixmap();
            }
        }
        else if ( ui->actionSetRuler->isChecked() )
        {
            int ret = m_roiAdjust.TestAgainstFindLines( pt, m_pLabelImgDisplay->size(), m_nCapturePos,
                                                        m_scaleFactor, m_ptCapture, m_lineOne );
            if ( 0 == ret )
            {
                UpdatePixmap();
            }
            UpdateRulerMeasurement();
        }
    }
}
void MainWindow::mouseReleaseEvent( QMouseEvent * )
{
    if ( m_bCaptured )
    {
        if ( nullptr != m_pRubberBand )
        {
            if ( ui->actionSetROI->isChecked() )
            {
                m_rectROI.setLeft( qRound( static_cast< double >( m_rectRubberBand.left() ) / m_scaleFactor ) );
                m_rectROI.setTop( qRound( static_cast< double >( m_rectRubberBand.top() ) / m_scaleFactor ) );
                m_rectROI.setRight( qRound( static_cast< double >( m_rectRubberBand.right() ) / m_scaleFactor ) );
                m_rectROI.setBottom( qRound( static_cast< double >( m_rectRubberBand.bottom() ) / m_scaleFactor ) );
            }
        }
        m_bCaptured = false;
        m_nCapturePos = 0;
    }

    if ( ui->actionSetRuler->isChecked() )
    {
        QString strMsg;
        ui->textEdit_msgs->setText( strMsg );
    }
}
void MainWindow::mouseDoubleClickEvent( QMouseEvent *pEvent )
{
    QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, pEvent->pos() );
    pt.setY( pt.y() - ui->mainToolBar->height() );
    int nX = qRound( ( static_cast< double >( pt.x() ) / m_scaleFactor ) + 0.5 );
    int nY = qRound( ( static_cast< double >( pt.y() ) / m_scaleFactor ) + 0.5 );
    if ( 0 <= nX && m_imgWidth > nX && 0 <= nY && m_imgHeight > nY )
    {
        if ( ui->actionSetROI->isChecked() )
            ui->actionSetROI->setChecked( false );
        else if ( ui->actionSetRuler->isChecked() )
            ui->actionSetRuler->setChecked( false );
        else if ( ui->actionSetSearchPoly->isChecked() )
            ui->actionSetSearchPoly->setChecked( false );
        else
            ui->actionSetROI->setChecked( true );
    }
}
size_t MainWindow::GetImagesPathsFromFolder( const QString strPath )
{
    m_imageFilePaths.clear();

    QDir folder;
    QStringList filters;
    filters << "*.png" << "*.jpg" << "*.tif";
    folder.setPath( strPath );
    QStringList listImages = folder.entryList( filters );
    if ( listImages.isEmpty() )
    {
        ui->textEdit_msgs->setText( "No images in folder" );
    }
    else
    {
        QString strFilepath, strFolder = folder.path();
        if ( !strFolder.endsWith( '/' ) )
            strFolder += '/';
        for ( int i = 0; i < listImages.count(); ++i )
        {
            strFilepath = strFolder + listImages.at( i );
            m_imageFilePaths.push_back( strFilepath.toStdString() );
        }
    }
    return m_imageFilePaths.size();
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// other control event handling
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::on_lineEdit_imageFolder_textEdited( const QString &strPath )
{
    ui->listWidget_imageFolder->clear();
    size_t imgCount = GetImagesPathsFromFolder( strPath );
    if ( 0 < imgCount )
    {
        ui->listWidget_imageFolder->clear();
        for ( size_t i = 0; i < m_imageFilePaths.size(); ++i )
        {
            fs::path p( m_imageFilePaths[ i ] );
            if ( fs::exists( p ) )
            {
                ui->listWidget_imageFolder->addItem( QString::fromStdString( p.filename().string() ) );
            }
        }
        if ( 0 < ui->listWidget_imageFolder->count() )
            ui->listWidget_imageFolder->setCurrentRow( 0 );
        on_actionZoomToFit_triggered();
    }
#if WIN32
    if ( ui->lineEdit_imageFolder->text().toLower() == ui->lineEdit_findLineTopFolder->text().toLower() )
#else
    if ( ui->lineEdit_imageFolder->text() == ui->lineEdit_findLineTopFolder->text() )
#endif
    {
        ui->lineEdit_folderMatch_status->setText( "Folders match");
        ui->lineEdit_folderMatch_status->setStyleSheet("QLineEdit {background-color: green; color: white;}");
        ui->lineEdit_folderMatch_status->setToolTip("The explore folder matches the find line folder.");
    }
    else
    {
        ui->lineEdit_folderMatch_status->setText( "CAUTION: Folders do not match");
        ui->lineEdit_folderMatch_status->setStyleSheet("QLineEdit {background-color: yellow; color: black;}");
        ui->lineEdit_folderMatch_status->setToolTip("Before processing image folder(s), check the Line find folder location. The explore folder does not match the find line folder.");
    }
}
void MainWindow::on_lineEdit_findLineTopFolder_textEdited(const QString)
{
#if WIN32
    if ( ui->lineEdit_imageFolder->text().toLower() == ui->lineEdit_findLineTopFolder->text().toLower() )
#else
    if ( ui->lineEdit_imageFolder->text() == ui->lineEdit_findLineTopFolder->text() )
#endif
    {
        ui->lineEdit_folderMatch_status->setText( "Folders match");
        ui->lineEdit_folderMatch_status->setStyleSheet("QLineEdit {background-color: green; color: white;}");
        ui->lineEdit_folderMatch_status->setToolTip("The explore folder matches the find line folder.");
    }
    else
    {
        ui->lineEdit_folderMatch_status->setText( "CAUTION: Folders do not match");
        ui->lineEdit_folderMatch_status->setStyleSheet("QLineEdit {background-color: yellow; color: black;}");
        ui->lineEdit_folderMatch_status->setToolTip("Before processing image folder(s), check the Line find folder location. The explore folder does not match the find line folder.");
    }
}
void MainWindow::on_listWidget_imageFolder_currentRowChanged( int row )
{
    if ( 0 <= row )
    {
        QString strFolder = ui->lineEdit_imageFolder->text();
        if ( !strFolder.endsWith( '/') )
            strFolder += '/';

        QString strFilepath = strFolder + ui->listWidget_imageFolder->item( row )->text();
        QFileInfo fileInfo( strFilepath );

        GC_STATUS retVal = m_visApp.LoadImageToApp( strFilepath.toStdString() );
        if ( GC_WARN == retVal )
        {
            cv::Size sizeImg;
            retVal = m_visApp.GetImageSize( sizeImg );
            if ( 0 == retVal )
            {
                int ret = ResizeImage( sizeImg.width, sizeImg.height );
                ui->statusBar->showMessage( 0 == ret ? QString( "Image resized" ) : QString( "Could not resize image" ) );
                if ( 0 != ret )
                    retVal = GC_ERR;
            }
        }
        if ( GC_ERR == retVal )
            ui->statusBar->showMessage( "Could not load image: " + strFilepath );
        on_horizontalSlider_zoom_valueChanged( ui->horizontalSlider_zoom->value() );
        if ( ui->checkBox_showFindLine->isChecked() && !m_visApp.isRunningFindLine() )
        {
            on_pushButton_findLineCurrentImage_clicked();
        }
        UpdatePixmapTarget();
    }
}
void MainWindow::on_horizontalSlider_zoom_valueChanged( int )
{
    m_scaleFactor = static_cast< double >( ui->horizontalSlider_zoom->value() ) / 100.0;
    ScaleImage();
}
void MainWindow::on_actionZoomToFit_triggered() { ZoomTo( m_imgWidth, m_imgHeight ); }
void MainWindow::on_actionZoom100_triggered() { ui->horizontalSlider_zoom->setValue( 100 ); }
void MainWindow::UpdateRulerMeasurement()
{
//    QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, m_lineOne.p1() );
//    pt.setY( pt.y() - ui->mainToolBar->height() );
    int nX1 = qRound( static_cast< double >( m_lineOne.p1().x() ) / 1.0 );
    int nY1 = qRound( static_cast< double >( m_lineOne.p1().y() ) / 1.0 );

//    pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, m_lineOne.p2() );
//    pt.setY( pt.y() - ui->mainToolBar->height() );
    int nX2 = qRound( static_cast< double >( m_lineOne.p2().x() ) / 1.0 );
    int nY2 = qRound( static_cast< double >( m_lineOne.p2().y() ) / 1.0 );

    double lenPix = Distance( nX1, nY1, nX2, nY2 );

    cv::Point2d world1, world2;
    GC_STATUS retVal1 = m_visApp.PixelToWorld( cv::Point2d( static_cast< double >( nX1 ), static_cast< double >( nY1 ) ), world1 );
    if ( GC_OK != retVal1 )
    {
        world1 = cv::Point2d( -9999999.9, -9999999.9 );
    }
    GC_STATUS retVal2 = m_visApp.PixelToWorld( cv::Point2d( static_cast< double >( nX2 ), static_cast< double >( nY2 ) ), world2 );
    if ( GC_OK != retVal2 )
    {
        world2 = cv::Point2d( -9999999.9, -9999999.9 );
    }
    double lenWorld = ( GC_OK != retVal1 || GC_OK != retVal2 ) ? -9999999.9 : Distance( world1.x, world1.y, world2.x, world2.y );

    char buf[ 32 ];
    sprintf( buf, "Len Pix=%0.1f Wor=%0.3f", lenPix, lenWorld);
    ui->textEdit_measures->setText( buf );
    sprintf( buf, "w1(%0.1f, %0.1f) w2(%0.1f, %0.1f)", world1.x, world1.y, world2.x, world2.y );
    ui->textEdit_measures->append( buf );
    sprintf( buf, "p1(%d, %d) p2(%d, %d)", nX1, nY1, nX2, nY2);
    ui->textEdit_measures->append( buf );
}
void MainWindow::UpdateRegionButton()
{
    bool enableResetRegionButton = true;
    if ( ui->actionSetROI->isChecked() )
    {
        ui->actionSetRuler->setChecked( false );
        ui->actionSetSearchPoly->setChecked( false );
    }
    else
    {
        enableResetRegionButton = false;
    }
    ui->pushButton_resetSearchRegion->setEnabled( enableResetRegionButton );
    ScaleImage();
}
void MainWindow::UpdatePolyButton()
{
    bool enableResetRegionButton = true;
    if ( ui->actionSetSearchPoly->isChecked() )
    {
        ui->actionSetROI->setChecked( false );
        ui->actionSetRuler->setChecked( false );
    }
    else
    {
        enableResetRegionButton = false;
    }
    ui->pushButton_resetSearchRegion->setEnabled( enableResetRegionButton );
    ScaleImage();
}
void MainWindow::UpdateRulerButton()
{
    bool enableResetRegionButton = true;
    if ( ui->actionSetRuler->isChecked() )
    {
        ui->actionSetROI->setChecked( false );
        ui->actionSetSearchPoly->setChecked( false );
    }
    else
    {
        enableResetRegionButton = false;
    }
    ui->pushButton_resetSearchRegion->setEnabled( enableResetRegionButton );
    ScaleImage();
}
void MainWindow::on_actionSetROI_toggled( bool ) { UpdateRegionButton(); }
void MainWindow::on_actionSetSearchPoly_toggled( bool ) { UpdatePolyButton(); }
void MainWindow::on_actionSetRuler_toggled( bool ) { UpdateRulerButton(); }
void MainWindow::on_actionImageLoad_triggered()
{
    QString filters = "Image Files (*.png *.jpg *.bmp)";
    QString strFullPath = QFileDialog::getOpenFileName( this,
        "Open Image", m_folderLoadImages, "Image Files (*.png *.jpg *.bmp)", &filters );
    if ( !strFullPath.isEmpty() )
    {
        QDir dirInfo( strFullPath );
        QFileInfo fileInfo( strFullPath );
        m_folderLoadImages = dirInfo.absolutePath();
        int ret = m_visApp.LoadImageToApp( strFullPath.toStdString() );
        if ( 1 == ret )
        {
            cv::Size sizeImg;
            ret = m_visApp.GetImageSize( sizeImg );
            if ( 0 == ret )
            {
                ret = ResizeImage( sizeImg.width, sizeImg.height );
            }
            if ( 0 != ret )
                ui->statusBar->showMessage( "Could not resize image" );
        }
        if ( 0 != ret )
        {
            ui->statusBar->showMessage( "Could not load image: " + strFullPath );
        }
        else
        {
            on_actionZoom100_triggered();

            cv::Size sizeImg;
            ret = m_visApp.GetImageSize( sizeImg );
            if ( 0 != ret )
            {
                ui->statusBar->showMessage( "Get image size failed (on image load)" );
                if ( sizeImg.width != m_imgWidth || sizeImg.height != m_imgWidth )
                {
                    ret = ResizeImage( sizeImg.width, sizeImg.height );
                }
            }
        }
        if ( 0 == ret )
            ui->statusBar->showMessage( "Loaded: " + strFullPath );
    }
}
void MainWindow::on_actionImageSave_triggered()
{
    QString filters = "Image Files (*.png *.jpg *.bmp)";
    QString strFullPath = QFileDialog::getSaveFileName( this,
        "Save Image", m_folderSaveImages, "Image Files (*.png *.jpg *.bmp)", &filters );

    if ( !strFullPath.isEmpty() )
    {
        int ret = 0;
        QDir dirInfo( strFullPath );
        m_folderSaveImages = dirInfo.absolutePath();

        int nColorType = -1;
        if ( 0 == m_pComboBoxImageToView->currentText().compare( "Grayscale") )
            nColorType = BUF_GRAY;
        else if ( 0 == m_pComboBoxImageToView->currentText().compare( "Overlay") )
            nColorType = BUF_OVERLAY;
        else if ( 0 == m_pComboBoxImageToView->currentText().compare( "Color") )
            nColorType = BUF_RGB;
        else
        {
            ui->statusBar->showMessage( "Invalid color type selected for save" );
            ret = -1;
        }

        if ( 0 == ret )
        {
            GC_STATUS retVal = m_visApp.SaveImage( strFullPath.toStdString(), static_cast< IMG_BUFFERS >( nColorType ) );
            if ( GC_OK != retVal )
                ui->statusBar->showMessage( "Save image failed" );
            else
                ui->statusBar->showMessage( "Saved: " + strFullPath );
        }
    }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// custom slots
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// button handlers
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::on_toolButton_clearMsgs_clicked() { ui->textEdit_msgs->clear(); }
void MainWindow::on_toolButton_imageFolder_browse_clicked()
{
    QString strFullPath = QFileDialog::getExistingDirectory( this, "Select image source folder", m_folderLoadImages );

    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "No folder selected" );
    }
    else
    {
        QDir dirInfo( strFullPath );
        m_folderLoadImages = dirInfo.absolutePath();
        ui->lineEdit_imageFolder->setText( strFullPath );
        on_lineEdit_imageFolder_textEdited( strFullPath );
    }
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// vision calibration
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::on_pushButton_visionCalibrateLoad_clicked()
{
    // QString strFullPath = QFileDialog::getSaveFileName( this, "Set calibration json filepath", ui->lineEdit_calibVisionResult_json->text() );
    QString strFullPath = QFileDialog::getOpenFileName( this, "Set calibration json filepath", ui->lineEdit_calibVisionResult_json->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not set calib result json file" );
    }
    else if (! strFullPath.contains( ".json" ) )
    {
        ui->statusBar->showMessage( "File must have \".json\" extension" );
    }
    else
    {
        ui->lineEdit_calibVisionResult_json->setText( strFullPath );
        GC_STATUS retVal = m_visApp.LoadCalib( ui->lineEdit_calibVisionResult_json->text().toStdString() );
        if ( GC_OK == retVal )
        {
            m_bCalibDirty = false;
            ui->checkBox_showCalib->setChecked( true );
            ui->checkBox_showSearchROI->setChecked( true );
            ui->checkBox_calibSearchROI->setChecked( true );
            m_pComboBoxImageToView->setCurrentText( "Overlay" );
            UpdateTargetRect();
            UpdateGUIEnables();
            UpdateCalibType();
            UpdateCalibSearchRegion();
        }
        else
        {
            ui->textEdit_msgs->append( "Could not load calibration from " + ui->lineEdit_calibVisionResult_json->text() );
        }
    }
}
void MainWindow::on_pushButton_visionCalibrateSave_clicked()
{
    string strFullPath = ui->lineEdit_calibVisionResult_json->text().toStdString();
    GC_STATUS retVal = m_visApp.SaveCalib( strFullPath );
    if ( GC_OK == retVal )
    {
        m_bCalibDirty = false;
        ui->textEdit_msgs->setText( "Calibration save SUCCESS" );
        ui->statusBar->showMessage( "Calibration save SUCCESS" );
    }
    else
    {
        ui->textEdit_msgs->setText( "Calibration save FAILURE" );
        ui->statusBar->showMessage( "Calibration save FAILURE" );
    }
}
void MainWindow::on_pushButton_visionCalibrateSaveAs_clicked()
{
    QString strFullPath = QFileDialog::getSaveFileName( this, "Get calibration json filepath", ui->lineEdit_calibVisionResult_json->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not get calib result json file" );
    }
    else if (! strFullPath.contains( ".json" ) )
    {
        ui->statusBar->showMessage( "File must have \".json\" extension" );
    }
    else
    {
        GC_STATUS retVal = m_visApp.SaveCalib( strFullPath.toStdString() );
        if ( GC_OK == retVal )
        {
            m_bCalibDirty = false;
            ui->lineEdit_calibVisionResult_json->setText( strFullPath );
            ui->textEdit_msgs->setText( "Calibration save as SUCCESS" );
            ui->statusBar->showMessage( "Calibration save as SUCCESS" );
        }
        else
        {
            ui->textEdit_msgs->setText( "Calibration save as FAILURE" );
            ui->statusBar->showMessage( "Calibration save as FAILURE" );
        }
    }
}
void MainWindow::on_pushButton_visionCalibrate_clicked()
{
    ui->checkBox_showCalib->setChecked( false );
    QString strFolder = ui->lineEdit_imageFolder->text();
    if ( !strFolder.endsWith( '/') )
        strFolder += '/';

    QString strFilepath = strFolder + ui->listWidget_imageFolder->currentItem()->text();
    ui->textEdit_msgs->setText( "calibrating..." );
    ui->textEdit_msgs->update();
    ui->statusBar->showMessage( "calibrating..." );
    ui->statusBar->update();

    UpdateCalibSearchRegion();

    CalibJsonItems calibItems( ui->lineEdit_calibVisionResult_json->text().toStdString(),
                               ui->checkBox_calibSearchROI->isChecked(),
                               cv::Rect( m_rectROI.x(), m_rectROI.y(), m_rectROI.width(), m_rectROI.height() ),
                               ui->doubleSpinBox_octagonFacetLength->value(), ui->doubleSpinBox_octagonZeroOffset->value(),
                               LineSearchRoi( cv::Point( m_lineSearchPoly.lftTop.x(), m_lineSearchPoly.lftTop.y() ),
                                              cv::Point( m_lineSearchPoly.rgtTop.x(), m_lineSearchPoly.rgtTop.y() ),
                                              cv::Point( m_lineSearchPoly.lftBot.x(), m_lineSearchPoly.lftBot.y() ),
                                              cv::Point( m_lineSearchPoly.rgtBot.x(), m_lineSearchPoly.rgtBot.y() ) ) );

    string jsonControlStr;
    int ret = CalibExecutive::FormOctagonCalibJsonString( calibItems, jsonControlStr );
    if ( 0 != ret )
    {
        ui->statusBar->showMessage( "Find line: FAILURE -- could not create octagon calib control string" );
    }
    GC_STATUS retVal = GC_OK;
    retVal = m_visApp.Calibrate( strFilepath.toStdString(), jsonControlStr );
    if ( GC_OK != retVal )
    {
        ui->statusBar->showMessage( "Calibration: FAILURE" );
    }
    else
    {
        m_bCalibDirty = true;
        ui->checkBox_showCalib->setChecked( true );
        ui->statusBar->showMessage( "Calibration: SUCCESS" );
    }
    ui->label_calibStatus->setText( "Calibration JSON file" + QString( m_bCalibDirty ? " (dirty)" : "" ) );
    m_pComboBoxImageToView->setCurrentText( "Overlay" );
    UpdatePixmapTarget();
}
void MainWindow::on_pushButton_resetSearchRegion_clicked()
{
    if ( ui->actionSetROI->isChecked() )
    {
        cv::Size imgSize;
        m_visApp.GetImageSize( imgSize );
        m_rectROI = QRect( imgSize.width / 10, imgSize.height / 10, imgSize.width >> 2, imgSize.height >> 2 );
        ScaleImage();
    }
    else if ( ui->actionSetSearchPoly->isChecked() )
    {
        cv::Size imgSize;
        m_visApp.GetImageSize( imgSize );
        int width = imgSize.width >> 1;
        int height = imgSize.height >> 1;
        int lft = imgSize.width / 10;
        int top = imgSize.height / 10;
        m_lineSearchPoly = LineSearchPoly( QPoint( lft, top ), QPoint( lft + width, top ),
                                            QPoint( lft + width, top + height ), QPoint( lft, top + height ) );
        ScaleImage();
    }
    else if ( ui->actionSetRuler->isChecked() )
    {
        cv::Size imgSize;
        m_visApp.GetImageSize( imgSize );
        m_lineOne = QLine( QPoint( m_scaleFactor * imgSize.width / 10, m_scaleFactor * imgSize.height / 10 ),
                           QPoint( m_scaleFactor * ( imgSize.width / 10 + ( imgSize.width >> 1 ) ),
                                   m_scaleFactor * ( imgSize.height / 10 + ( imgSize.height >> 1 ) ) ) );
        UpdatePixmap();
        UpdateRulerMeasurement();
    }
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// vision findline
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::on_toolButton_findLineTopFolder_browse_clicked()
{
    QString strFullPath = QFileDialog::getExistingDirectory( this, "Set annotated result image folder", ui->lineEdit_findLineTopFolder->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not get annotated result image folder path" );
    }
    else
    {
        ui->lineEdit_findLineTopFolder->setText( strFullPath );
    }
}
void MainWindow::on_pushButton_setFindLineFolderFromExploreFolder_clicked()
{
    ui->lineEdit_findLineTopFolder->setText( ui->lineEdit_imageFolder->text() );
    on_lineEdit_findLineTopFolder_textEdited( ui->lineEdit_findLineTopFolder->text() );
}
void MainWindow::on_toolButton_findLine_resultCSVFile_browse_clicked()
{
    QString strFullPath = QFileDialog::getSaveFileName( this,
            "Select find water level result csv file", ui->lineEdit_findLine_resultCSVFile->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "No find water level result csv file selected" );
    }
    else if ( !strFullPath.contains( ".csv" ) )
    {
        ui->statusBar->showMessage( "File must have \".csv\" extension" );
    }
    else
    {
        ui->lineEdit_findLine_resultCSVFile->setText( strFullPath );
    }
}
void MainWindow::on_toolButton_saveSearchROIFolder_browse_clicked()
{
    QString strFullPath = QFileDialog::getExistingDirectory( this, "Set waterline search roi folder", ui->lineEdit_saveSearchROIFolder->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not set waterline search roi folder path" );
    }
    else
    {
        ui->lineEdit_saveSearchROIFolder->setText( strFullPath );
    }
}
void MainWindow::on_toolButton_findLine_annotatedResultFolder_browse_clicked()
{
    QString strFullPath = QFileDialog::getExistingDirectory( this, "Set annotated result image folder", ui->lineEdit_findLine_annotatedResultFolder->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not set annotated result image folder path" );
    }
    else
    {
        ui->lineEdit_findLine_annotatedResultFolder->setText( strFullPath );
    }
}
void MainWindow::on_pushButton_findLineCurrentImage_clicked()
{
    GC_STATUS retVal = GC_OK;
    if ( m_visApp.isRunningFindLine() )
    {
        ui->statusBar->showMessage( "WARNING: Cannot run find line when find line folder run is active" );
        retVal = GC_WARN;
    }
    else
    {
        FindLineParams params;
        string folder = ui->lineEdit_imageFolder->text().toStdString();
        if ( '/' != folder[ folder.size() - 1 ] )
            folder += '/';
        params.imagePath = folder + ui->listWidget_imageFolder->currentItem()->text().toStdString();
        params.resultCSVPath.clear();
        params.resultImagePath.clear();
        params.lineSearchROIFolder.clear();
        params.calibFilepath = ui->lineEdit_calibVisionResult_json->text().toStdString();
        params.timeStampFormat = ui->lineEdit_timestampFormat->text().toStdString();
        params.timeStampType = ui->radioButton_dateTimeInFilename->isChecked() ? FROM_FILENAME : FROM_EXIF;
        params.timeStampStartPos = ui->spinBox_timeStringPosZero->value();
        params.octagonZeroOffset = ui->doubleSpinBox_octagonZeroOffset->value();
        params.calibControlString.clear();
        params.lineSearchROIFolder = ui->groupBox_enableSaveSearchROI->isChecked() ? ui->lineEdit_saveSearchROIFolder->text().toStdString() : "";
        if ( params.isOctagonCalib )
        {
            CalibJsonItems calibItems( ui->lineEdit_calibVisionResult_json->text().toStdString(), ui->checkBox_calibSearchROI->isChecked(),
                                       cv::Rect( m_rectROI.x(), m_rectROI.y(), m_rectROI.width(), m_rectROI.height() ),
                                       ui->doubleSpinBox_octagonFacetLength->value(), ui->doubleSpinBox_octagonZeroOffset->value(),
                                       LineSearchRoi( cv::Point( m_lineSearchPoly.lftTop.x(), m_lineSearchPoly.lftTop.y() ),
                                                      cv::Point( m_lineSearchPoly.rgtTop.x(), m_lineSearchPoly.rgtTop.y() ),
                                                      cv::Point( m_lineSearchPoly.lftBot.x(), m_lineSearchPoly.lftBot.y() ),
                                                      cv::Point( m_lineSearchPoly.rgtBot.x(), m_lineSearchPoly.rgtBot.y() ) ) );
            int ret = CalibExecutive::FormOctagonCalibJsonString( calibItems, params.calibControlString );
            if ( 0 != ret )
            {
                ui->statusBar->showMessage( "Find line: FAILURE -- could not create octagon calib control string" );
            }
        }

        FindLineResult result;
        retVal = m_visApp.CalcLine( params, result );
        if ( GC_OK == retVal )
        {
            ui->checkBox_showFindLine->setChecked( true );
            m_pComboBoxImageToView->setCurrentText( "Overlay" );
            ui->statusBar->showMessage( "Find line: SUCCESS" );
        }
        else
        {
            ui->statusBar->showMessage( "Find line: FAILURE" );
        }

        UpdatePixmapTarget();

        ui->textEdit_msgs->clear();
        for ( size_t i = 0; i < result.msgs.size(); ++i )
        {
            ui->textEdit_msgs->append( QString::fromStdString( result.msgs[ i ] ) );
        }
        ui->textEdit_msgs->append( ui->lineEdit_saveSearchROIFolder->text() + ( ui->checkBox_saveSearchROI->isChecked() ? "true" : "false " ) );
    }
}
void MainWindow::on_pushButton_findLine_processFolder_clicked()
{
    FindLineParams params;
    string folder = ui->lineEdit_findLineTopFolder->text().toStdString();
    if ( '/' != folder[ folder.size() - 1 ] )
        folder += '/';
    params.isOctagonCalib = true;
    params.calibFilepath = ui->lineEdit_calibVisionResult_json->text().toStdString();
    params.timeStampFormat = ui->lineEdit_timestampFormat->text().toStdString();
    params.timeStampType = ui->radioButton_dateTimeInFilename->isChecked() ? FROM_FILENAME : FROM_EXIF;
    params.timeStampStartPos = ui->spinBox_timeStringPosZero->value();
    params.resultImagePath = ui->checkBox_createFindLine_annotatedResults->isChecked() ? ui->lineEdit_findLine_annotatedResultFolder->text().toStdString() : "";
    params.resultCSVPath = ui->checkBox_createFindLine_csvResultsFile->isChecked() ? ui->lineEdit_findLine_resultCSVFile->text().toStdString() : "";
    params.lineSearchROIFolder = ui->checkBox_saveSearchROI->isChecked() ? ui->lineEdit_saveSearchROIFolder->text().toStdString() : "";

    ui->pushButton_findLine_processFolder->setEnabled( false );
    ui->pushButton_findLine_stopFolderProcess->setEnabled( true );

    vector< string > headings = { "filename", "timestamp", "water level" };
    InitTable( headings );

    GC_STATUS retVal = m_visApp.CalcLinesInFolder( folder, params, ui->radioButton_folderOfImages->isChecked() );

    ui->textEdit_msgs->clear();
    ui->textEdit_msgs->append( GC_OK == retVal ? "Folder run started" : "Folder run failed to start" );

    on_actionZoomToFit_triggered();
    m_pComboBoxImageToView->setCurrentText( "Color" );
    ui->checkBox_showFindLine->setChecked( true );
}

void MainWindow::on_pushButton_findLine_stopFolderProcess_clicked()
{
    if ( m_visApp.isRunningFindLine() )
    {
        GC_STATUS retVal = m_visApp.CalcLinesThreadFinish();
        if ( GC_OK == retVal )
        {
            ui->pushButton_findLine_processFolder->setEnabled( true );
            ui->pushButton_findLine_stopFolderProcess->setEnabled( false );
            if ( 0 == ui->listWidget_imageFolder->count() )
                m_pQImg->fill( Qt::black );
            else
                ui->listWidget_imageFolder->setCurrentRow( 0 );
        }
        QString msg = QString( "Find lines in folder stop attempt:" ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" );
        ui->textEdit_msgs->clear();
        ui->textEdit_msgs->append( msg );
    }
    else
    {
        ui->textEdit_msgs->append( "Tried to stop folder line fine process when it was not running" );
    }
}
void MainWindow::on_pushButton_showImageMetadata_clicked()
{
    QString strFullPath;
    if ( -1 == ui->listWidget_imageFolder->currentRow() )
    {
        strFullPath = QFileDialog::getOpenFileName( this, "Select image", ui->lineEdit_imageFolder->text(), "Image Files (*.png *.jpg)" );
    }
    else
    {
        string folder = ui->lineEdit_imageFolder->text().toStdString();
        if ( '/' != folder[ folder.size() - 1 ] )
            folder += '/';
        strFullPath = QString( folder.c_str() ) + ui->listWidget_imageFolder->currentItem()->text();
    }
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not get image to show metadata" );
    }
    else
    {
        QString lower = strFullPath.toLower();
        if ( lower.endsWith( ".jpg" ) || lower.endsWith( ".png" ) )
        {
            std::string data;
            GC_STATUS retVal = m_visApp.GetMetadata( strFullPath.toStdString(), data );
            ui->textEdit_msgs->setText( data.c_str() );
            ui->textEdit_msgs->append( QString( GC_OK == retVal ? "SUCCESS" : "SOME OR ALL METADATA NOT AVAILABLE" ) );
        }
    }
}
void MainWindow::on_pushButton_showCalibration_clicked()
{
    std::string calibParams;
    GC_STATUS retVal = m_visApp.GetCalibParams( calibParams );
    if ( GC_OK != retVal )
    {
        ui->statusBar->showMessage( "Could not retrive calibraion parameters" );
    }
    else
    {
        ui->textEdit_msgs->setText( QString( calibParams.c_str() ) );
    }
}
void MainWindow::ClearTable()
{
    QAbstractItemModel *const mdl = ui->tableWidget->model();
    mdl->removeRows( 0, mdl->rowCount() );
    if ( !m_visApp.isRunningFindLine() )
        mdl->removeColumns( 0, mdl->columnCount() );
}
int MainWindow::InitTable( const vector< string > headings )
{
    int ret = 0;
    if ( 1 > headings.size() || 48 < headings.size() )
    {
        ui->statusBar->showMessage( "FAIL[Init]: Init column count must be in range 1-48" );
        ret = -1;
    }
    else
    {
        ClearTable();
        QStringList headerList;
        for ( size_t i = 0; i < headings.size(); ++i )
        {
            headerList.append( QString( headings[ i ].c_str() ) );
        }
        ui->tableWidget->setColumnCount( static_cast< int >( headings.size() ) );
        ui->tableWidget->setHorizontalHeaderLabels( headerList );
    }

    return ret;
}
int MainWindow::AddRow( const string row_string )
{
    int ret = 0;

    if ( row_string.empty() )
    {
        ui->statusBar->showMessage( "FAIL: No data in row string" );
        ret = -1;
    }
    else
    {
        vector< string > column_strings;
        size_t start = 0;
        for ( size_t i = 0; i < row_string.size(); ++i )
        {
            if ( ',' == row_string[ i ] )
            {
                column_strings.push_back( row_string.substr( start, ( i - start ) ) );
                start = i + 1;
            }
        }
        column_strings.push_back( row_string.substr( start ) );
        if ( static_cast< int >( column_strings.size() ) != ui->tableWidget->columnCount() )
        {
            ui->statusBar->showMessage( "FAIL: Column count does not equal row items" );
            ret = -1;
        }
        else
        {
            QTableWidgetItem *item;
            int rowToAdd = ui->tableWidget->rowCount();
            ui->tableWidget->insertRow( rowToAdd );
            for ( size_t i = 0; i < column_strings.size(); ++i )
            {
                item = new QTableWidgetItem( column_strings[ i ].c_str() );
                ui->tableWidget->setItem( rowToAdd, static_cast< int >( i ), item );
            }
        }
    }

    return ret;
}
void MainWindow::on_pushButton_createAnimation_clicked()
{
    QString strFullPath;
    strFullPath = QFileDialog::getSaveFileName( this, "Select GIF filename", ui->lineEdit_imageFolder->text(), "Animations (*.gif *.GIF)" );
    if ( strFullPath.endsWith( ".gif" ) || strFullPath.endsWith( ".GIF" ) )
    {
        ui->pushButton_createAnimation->setEnabled( false );
        ui->pushButton_animationStop->setEnabled( true );
        ui->pushButton_findLine_processFolder->setEnabled( false );
        ui->pushButton_findLine_stopFolderProcess->setEnabled( false );

        GC_STATUS retVal = m_visApp.CreateAnimation( ui->lineEdit_imageFolder->text().toStdString(),
                                                     strFullPath.toStdString(), ui->spinBox_animateFPS->value(),
                                                     ui->doubleSpinBox_animateScale->value() );
        ui->textEdit_msgs->clear();
        ui->textEdit_msgs->append( GC_OK == retVal ? "Create GIF started" : "Create GIF failed to start" );
    }
    else
    {
        ui->textEdit_msgs->append( "Animation creation: Invalid extension. Must be .gif" );
    }
}
void MainWindow::on_pushButton_animationStop_clicked()
{
    if ( m_visApp.isRunningCreateGIF() )
    {
        GC_STATUS retVal = m_visApp.CreateGIFThreadFinish();
        if ( GC_OK == retVal )
        {
            ui->pushButton_createAnimation->setEnabled( true );
            ui->pushButton_animationStop->setEnabled( false );
            ui->pushButton_findLine_processFolder->setEnabled( true );
            ui->pushButton_findLine_stopFolderProcess->setEnabled( false );
        }
        QString msg = QString( "Create GIF stop attempt:" ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" );
        ui->textEdit_msgs->clear();
        ui->textEdit_msgs->append( msg );
    }
    else
    {
        ui->textEdit_msgs->append( "Tried to stop GIF create process when it was not running" );
    }
}
void MainWindow::on_pushButton_test_clicked()
{
    ui->statusBar->showMessage( "No test enabled" );
}
void MainWindow::on_pushButton_createCalibCommandLine_clicked()
{
    //--create_calib octagon --source "/media/kchapman/Elements/data/sunwater/image004.jpg" --calib_json "/media/kchapman/Elements/data/sunwater/2024_01_08_cal_images/calib_004.json" --result_image "/var/tmp/gaugecam/calib_result.png" --zero_offset 3.2 --facet_length 0.599 --waterline_roi 1059 529 1266 546 1266 858 1066 843 --calib_roi 502 271 399 446
    string msg = "--create_calib octagon ";
    string folder = ui->lineEdit_imageFolder->text().toStdString();
    if ( '/' != folder[ folder.size() - 1 ] )
        folder += '/';
    msg += "--source \"" + folder + ui->listWidget_imageFolder->currentItem()->text().toStdString() + "\" ";
    msg += "--calib_json \"" + ui->lineEdit_calibVisionResult_json->text().toStdString() + "\" ";
    msg += "--result_image \"\" ";
    msg += "--zero_offset " + to_string(ui->doubleSpinBox_octagonZeroOffset->value()) + " ";
    msg += "--facet_length " + to_string(ui->doubleSpinBox_octagonFacetLength->value()) + " ";
    msg += "--waterline_roi " + to_string(m_lineSearchPoly.lftTop.x()) + " " + to_string(m_lineSearchPoly.lftTop.y()) + " ";
    msg += to_string(m_lineSearchPoly.rgtTop.x()) + " " + to_string(m_lineSearchPoly.rgtTop.y()) + " ";
    msg += to_string(m_lineSearchPoly.lftBot.x()) + " " + to_string(m_lineSearchPoly.lftBot.y()) + " ";
    msg += to_string(m_lineSearchPoly.rgtBot.x()) + " " + to_string(m_lineSearchPoly.rgtBot.y()) + " ";
    msg += "--calib_roi " + to_string( m_rectROI.x() ) + " " + to_string( m_rectROI.y() ) +" ";
    msg += to_string( m_rectROI.width() ) + " " + to_string( m_rectROI.height() );
    ui->textEdit_msgs->setText( QString( msg.c_str() ) );
}
void MainWindow::on_pushButton_createFindCommandLine_clicked()
{
    if ( ui->radioButton_folderOfImages->isChecked() )
    {
        // --run_folder --timestamp_from_filename --timestamp_start_pos 10 --timestamp_format "yy-mm-dd-HH-MM" --source "./config/2012_demo/06/" --calib_json "./config/calib.json" --csv_file "/var/tmp/gaugecam/folder.csv" --result_folder "/var/tmp/gaugecam/"
        string msg = "--run_folder ";
        msg += ui->radioButton_dateTimeInFilename->isChecked() ? "--timestamp_from_filename " : "--timestamp_from_exif ";
        msg += "--timestamp_start_pos " + to_string( ui->spinBox_timeStringPosZero->value() ) + " ";
        msg += "--timestamp_format " + ui->lineEdit_timestampFormat->text().toStdString() + " ";
        msg += "--source " + ui->lineEdit_findLineTopFolder->text().toStdString() + " ";
        msg += "--calib_json " + ui->lineEdit_calibVisionResult_json->text().toStdString() + " ";
        if ( ui->checkBox_createFindLine_csvResultsFile->isChecked() )
            msg += "--csv_file " + ui->lineEdit_findLine_resultCSVFile->text().toStdString() + " ";
        if ( ui->checkBox_createFindLine_annotatedResults->isChecked() )
            msg += "--result_folder " + ui->lineEdit_findLine_annotatedResultFolder->text().toStdString() + " ";
        if ( ui->checkBox_saveSearchROI->isChecked() )
        {
            msg += "--line_roi_folder " + ui->lineEdit_saveSearchROIFolder->text().toStdString() + " ";
        }
        ui->textEdit_msgs->setText( QString( msg.c_str() ) );
    }
    else
    {
        ui->textEdit_msgs->setText( "Nested folder command line option not available" );
    }
}

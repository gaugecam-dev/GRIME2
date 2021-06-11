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

#include <list>
#include <string>
#include <iostream>
#include <QDir>
#include <QSettings>
#include <QMouseEvent>
#include <QTextStream>
#include <QPainter>
#include <QCursor>
#include <QColor>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QMessageBox>
#include <boost/signals2.hpp>
#include <boost/bind/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/filesystem.hpp>
#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
using namespace boost;
namespace fs = boost::filesystem;

#ifdef WIN32
static const QString __CONFIGURATION_FOLDER = "c:/gaugecam/config/";
static const QString __SETTINGS_FILEPATH = "c:/gaugecam/config/settingsWin.cfg";
#else
static const QString __CONFIGURATION_FOLDER = "./config/";
static const QString __SETTINGS_FILEPATH = "./config/settings.cfg";
#endif

static double DistToLine( const double dX, const double dY,
                          const double dX1, const double dY1,
                          const double dX2, const double dY2 )
{
    double dNum = fabs( ( dX2 - dX1 ) * ( dY1 - dY ) - ( dX1 - dX ) * ( dY2 - dY1 ) );
    double dDenom = sqrt( ( dX2 - dX1 ) * ( dX2 - dX1 ) + ( dY2 - dY1 ) * ( dY2 - dY1 ) );
    return 0.0 == dDenom ? 0.0 : dNum / dDenom;
}
static double Distance( const double x1, const int y1, const int x2, const int y2 )
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
    m_rectRubberBand( QRect( 0, 0, MAX_IMAGE_SIZE.width, MAX_IMAGE_SIZE.height ) )
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
    ret = m_visApp.Init( __CONFIGURATION_FOLDER.toStdString(), sizeImg );
    ui->textEdit_msgs->append( 0 == ret ? "Settings load succeeded" : "Settings load failed" );

    ui->actionSaveVideo->setEnabled( false );

    GC_STATUS retVal = m_visApp.LoadCalib( ui->lineEdit_calibVisionResult_json->text().toStdString() );
    if ( GC_OK != retVal )
    {
        ui->textEdit_msgs->append( "Could not load calibration from " + ui->lineEdit_calibVisionResult_json->text() );
    }

    createActions();
    createConnections();

    on_lineEdit_imageFolder_textEdited( ui->lineEdit_imageFolder->text() );
    on_actionZoom100_triggered();
    ui->widget_overlayCheckboxes->hide();
    UpdateGUIEnables();
}
MainWindow::~MainWindow()
{
    int ret = m_visApp.WriteSettings();
    if ( 0 != ret )
        QMessageBox::warning( this, "App write settings warning",
                              "FAIL:  Could not write application settings properly on program exit" );

    ret = WriteSettings( __SETTINGS_FILEPATH );
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
    connect( ui->checkBox_showCalib, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showFindLine, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showRowSums, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showDerivOne, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showDerivTwo, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showRANSAC, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showMoveROIs, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showMoveFind, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_showSearchROI, &QCheckBox::stateChanged, this, &MainWindow::UpdatePixmapTarget );
    connect( ui->checkBox_createFindLine_csvResultsFile, &QCheckBox::stateChanged, this, &MainWindow::UpdateGUIEnables );
    connect( ui->checkBox_createFindLine_annotatedResults, &QCheckBox::stateChanged, this, &MainWindow::UpdateGUIEnables );
    connect( ui->actionToggleControls, &QAction::toggled, this, &MainWindow::UpdateGUIEnables );

    connect( this, SIGNAL( sig_visAppMessage(QString) ), this, SLOT( do_visAppMessage(QString) ) );
    connect( this, SIGNAL( sig_updateProgess(int) ),     this, SLOT( do_updateProgress(int) ) );

    m_visApp.sigMessage.connect( bind( &MainWindow::on_visAppMessage, this, boost::placeholders::_1 ) );
    m_visApp.sigProgress.connect( bind( &MainWindow::on_updateProgress, this, boost::placeholders::_1 ) );
    m_visApp.sigTableAddRow.connect( bind( &MainWindow::on_tableAddRow, this, boost::placeholders::_1 ) );

    // feature recipe connections

}
int MainWindow::ResizeImage( const int width, const int height )
{
    if ( m_imgWidth == width && m_imgHeight == height && nullptr != m_pQImg )
        return 0;

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
        return -1;
    }
    m_pQImg->fill( Qt::black );

    m_pRubberBand = new QRubberBand( QRubberBand::Rectangle, m_pLabelImgDisplay );
    if ( nullptr == m_pRubberBand )
    {
        ui->statusBar->showMessage( "FAIL:  Could not instantiate rubberband selector" );
        return -1;
    }

    m_pRubberBand->setGeometry( m_rectRubberBand );
    return 0;
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

        m_folderLoadImages = pSettings->value( "loadFolder", "." ).toString();
        m_folderSaveImages = pSettings->value( "saveFolder", "." ).toString();

        ui->lineEdit_imageFolder->setText( pSettings->value( "imageFolder", m_folderLoadImages ).toString() );
        pSettings->endGroup();

        // vision stuff
        pSettings->beginGroup( "Vision" );
        ui->lineEdit_calibVisionTarget_csv->setText( pSettings->value( "calibCSVFileIn", __CONFIGURATION_FOLDER + "calibration_target_world_coordinates.csv" ).toString() );
        ui->lineEdit_calibVisionResult_json->setText( pSettings->value( "calibJsonFileOut", __CONFIGURATION_FOLDER + "calib.json" ).toString() );
        ui->lineEdit_findLineTopFolder->setText( pSettings->value( "findLineFolder", __CONFIGURATION_FOLDER ).toString() );
        ui->lineEdit_findLine_resultCSVFile->setText( pSettings->value( "findLineCSVOutPath", __CONFIGURATION_FOLDER + "waterlevel.csv" ).toString() );

        bool isFolderOfImages = pSettings->value( "folderOfImages", true ).toBool();
        ui->radioButton_folderOfImages->setChecked( isFolderOfImages );
        ui->radioButton_folderOfFolders->setChecked( !isFolderOfImages );

        bool createCSV = pSettings->value( "createCSVCheckbox", true ).toBool();
        ui->checkBox_createFindLine_csvResultsFile->setChecked( createCSV );

        ui->lineEdit_findLine_annotatedResultFolder->setText( pSettings->value( "findLineAnnotatedOutFolder", __CONFIGURATION_FOLDER ).toString() );
        ui->checkBox_createFindLine_annotatedResults->setChecked( pSettings->value( "createAnnotationCheckbox", false ).toBool() );
        ui->spinBox_timeStringPosZero->setValue( pSettings->value( "timestampStringStartPos", 5 ).toInt() );
        ui->radioButton_dateTimeInFilename->setChecked( pSettings->value( "timestampFromFilename", true ).toBool() );
        ui->radioButton_dateTimeInEXIF->setChecked( pSettings->value( "timestampFromEXIF", true ).toBool() );
        ui->lineEdit_timestampFormat->setText( pSettings->value( "timestampFormat", "yyyy-mm-ddTHH-MM" ).toString() );
        pSettings->endGroup();

        delete pSettings;
        pSettings = nullptr;
    }

    return ret;
}
int MainWindow::WriteSettings( const QString filepath )
{
    QSettings *pSettings = nullptr;
    if ( filepath.isEmpty() )
    {
        pSettings = new QSettings( "thrive", "VisTestApp" );
        if ( nullptr == pSettings )
        {
            QMessageBox::warning( this, "Write settings warning",
                                  "FAIL:  Could not instantiate read settings object, using defaults" );
            return -1;
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
            return -1;

        }
    }
    if ( nullptr == pSettings )
        return -1;

    // image stuff
    pSettings->beginGroup( "Image and ROI" );
    pSettings->setValue( "width", m_imgWidth );
    pSettings->setValue( "height", m_imgHeight );
    pSettings->setValue( "roiLeft", m_rectROI.left() );
    pSettings->setValue( "roiTop", m_rectROI.top() );
    pSettings->setValue( "roiWidth", m_rectROI.width() );
    pSettings->setValue( "roiHeight", m_rectROI.height() );

    pSettings->setValue( "loadFolder", m_folderLoadImages );
    pSettings->setValue( "saveFolder", m_folderSaveImages );

    pSettings->setValue( "imageFolder", ui->lineEdit_imageFolder->text() );
    pSettings->endGroup();

    // vision stuff
    pSettings->beginGroup( "Vision" );
    pSettings->setValue( "calibCSVFileIn", ui->lineEdit_calibVisionTarget_csv->text() );
    pSettings->setValue( "calibJsonFileOut", ui->lineEdit_calibVisionResult_json->text() );
    pSettings->setValue( "findLineFolder", ui->lineEdit_findLineTopFolder->text() );
    pSettings->setValue( "findLineCSVOutPath", ui->lineEdit_findLine_resultCSVFile->text() );
    pSettings->setValue( "folderOfImages", ui->radioButton_folderOfImages->isChecked() );
    pSettings->setValue( "createCSVCheckbox", ui->checkBox_createFindLine_csvResultsFile->isChecked() );
    pSettings->setValue( "findLineAnnotatedOutFolder", ui->lineEdit_findLine_annotatedResultFolder->text() );
    pSettings->setValue( "createAnnotationCheckbox", ui->checkBox_createFindLine_annotatedResults->isChecked() );
    pSettings->setValue( "timestampStringStartPos", ui->spinBox_timeStringPosZero->value() );
    pSettings->setValue( "timestampFromEXIF", ui->radioButton_dateTimeInEXIF->isChecked() );
    pSettings->setValue( "timestampFormat", ui->lineEdit_timestampFormat->text() );
    pSettings->endGroup();

    delete pSettings;
    pSettings = nullptr;

    return 0;
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
void MainWindow::TestAgainstFindLines( QPoint pt )
{
    QPoint ptTemp;
    ptTemp.setX( qRound( ( static_cast< double >( pt.x() ) / m_scaleFactor + 0.5 ) ) );
    ptTemp.setY( qRound( ( static_cast< double >( pt.y() ) / m_scaleFactor + 0.5 ) ) );
    switch ( m_nCapturePos )
    {
        case 1: m_lineOne.setP1( ptTemp ); break;
        case 2: m_lineOne.setP2( ptTemp ); break;
        case 5:
            m_lineOne.setP1( m_lineOne.p1() + ( ptTemp - m_ptCapture ) );
            m_lineOne.setP2( m_lineOne.p2() + ( ptTemp - m_ptCapture ) );
            break;
        default: break;
    }
    AdjustPointFindLines();
    m_ptCapture = ptTemp;
}
void MainWindow::TestAgainstRubberBands( QPoint pt )
{
    switch ( m_nCapturePos )
    {
        case 1:
            m_rectRubberBand.setLeft( pt.x() );
            m_rectRubberBand.setTop( pt.y() );
            break;
        case 2:
            m_rectRubberBand.setTop( pt.y() );
            break;
        case 4:
            m_rectRubberBand.setRight( pt.x() );
            m_rectRubberBand.setTop( pt.y() );
            break;
        case 8: m_rectRubberBand.setLeft( pt.x() ); break;
        case 16: m_rectRubberBand.setRight( pt.x() ); break;
        case 32:
            m_rectRubberBand.setLeft( pt.x() );
            m_rectRubberBand.setBottom( pt.y() );
            break;
        case 64:
            m_rectRubberBand.setBottom( pt.y() );
            break;
        case 128:
            m_rectRubberBand.setRight( pt.x() );
            m_rectRubberBand.setBottom( pt.y() );
            break;
        case 255: m_rectRubberBand.setCoords(
            m_rectRubberBand.left() - m_ptCapture.x() + pt.x(),
            m_rectRubberBand.top() - m_ptCapture.y() + pt.y(),
            m_rectRubberBand.right() - m_ptCapture.x() + pt.x(),
            m_rectRubberBand.bottom() - m_ptCapture.y() + pt.y() );
            break;
        default: break;
    }
    AdjustPointRubberBand();
    m_pRubberBand->setGeometry( m_rectRubberBand );
    m_rectROI.setLeft( qRound( static_cast< double >( m_rectRubberBand.left() ) / m_scaleFactor ) );
    m_rectROI.setTop( qRound( static_cast< double >( m_rectRubberBand.top() ) / m_scaleFactor ) );
    m_rectROI.setRight( qRound( static_cast< double >( m_rectRubberBand.right() ) / m_scaleFactor ) );
    m_rectROI.setBottom( qRound( static_cast< double >( m_rectRubberBand.bottom() ) / m_scaleFactor ) );
    m_ptCapture = pt;
}
void MainWindow::AdjustPointRubberBand()
{
    if ( 5 > m_rectRubberBand.width() ) m_rectRubberBand.setRight( m_rectRubberBand.x() + 5 );
    if ( 5 > m_rectRubberBand.height() ) m_rectRubberBand.setBottom( m_rectRubberBand.y() + 5 );
    if ( 0 > m_rectRubberBand.x() ) m_rectRubberBand.setLeft( 0 );
    if ( 0 > m_rectRubberBand.y() ) m_rectRubberBand.setTop( 0 );
    if ( m_pLabelImgDisplay->width() < m_rectRubberBand.left() + m_rectRubberBand.width() )
            m_rectRubberBand.setRight( m_pLabelImgDisplay->width() - 1 );
    if ( m_pLabelImgDisplay->height() < m_rectRubberBand.top() + m_rectRubberBand.height() )
            m_rectRubberBand.setBottom( m_pLabelImgDisplay->height() - 1 );
    if ( 5 > m_rectRubberBand.width() ) m_rectRubberBand.setLeft( m_rectRubberBand.right() - 5 );
    if ( 5 > m_rectRubberBand.height() ) m_rectRubberBand.setTop( m_rectRubberBand.bottom() - 5 );
}
void MainWindow::UpdateGUIEnables()
{
    ui->lineEdit_findLine_resultCSVFile->setEnabled( ui->checkBox_createFindLine_csvResultsFile->isChecked() );
    ui->toolButton_findLine_resultCSVFile_browse->setEnabled( ui->checkBox_createFindLine_csvResultsFile->isChecked() );
    ui->lineEdit_findLine_annotatedResultFolder->setEnabled( ui->checkBox_createFindLine_annotatedResults->isChecked() );
    ui->toolButton_findLine_annotatedResultFolder_browse->setEnabled( ui->checkBox_createFindLine_annotatedResults->isChecked() );
    ui->widget_overlayCheckboxes->setHidden( !ui->actionToggleControls->isChecked() );
}
void MainWindow::AdjustPointFindLines()
{
    int nWidth = qRound( static_cast< double >( m_pLabelImgDisplay->width() ) / m_scaleFactor + 0.5 ) - 5;
    int nHeight = qRound( static_cast< double >( m_pLabelImgDisplay->height() ) / m_scaleFactor + 0.5 ) - 5;
    QPoint ptLine1Pt1( m_lineOne.p1().x(), m_lineOne.p1().y() );
    QPoint ptLine1Pt2( m_lineOne.p2().x(), m_lineOne.p2().y() );
    if ( 5 > ptLine1Pt1.x() ) ptLine1Pt1.setX( 5 );
    if ( 5 > ptLine1Pt2.x() ) ptLine1Pt2.setX( 5 );
    if ( 5 > ptLine1Pt1.y() ) ptLine1Pt1.setY( 5 );
    if ( 5 > ptLine1Pt2.y() ) ptLine1Pt2.setY( 5 );
    if ( nWidth <= ptLine1Pt1.x() ) ptLine1Pt1.setX( nWidth - 1 );
    if ( nWidth <= ptLine1Pt2.x() ) ptLine1Pt2.setX( nWidth - 1 );
    if ( nHeight <= ptLine1Pt1.y() ) ptLine1Pt1.setY( nHeight - 1 );
    if ( nHeight <= ptLine1Pt2.y() ) ptLine1Pt2.setY( nHeight - 1 );
    m_lineOne.setP1( ptLine1Pt1 );
    m_lineOne.setP2( ptLine1Pt2 );
    UpdatePixmap();
}
void MainWindow::UpdatePixmapTarget()
{
#ifdef _WIN32
    update();
#else
    UpdatePixmap();
#endif
}
void MainWindow::UpdatePixmap()
{
    if ( nullptr == m_pQImg )
    {
        ui->statusBar->showMessage( "FAIL: Image display buffer not initialized" );
        return;
    }

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
        return;
    }

    IMG_DISPLAY_OVERLAYS overlays = static_cast< IMG_DISPLAY_OVERLAYS >(
                ( ui->checkBox_showCalib->isChecked() ? CALIB : OVERLAYS_NONE ) +
                ( ui->checkBox_showFindLine->isChecked() ? FINDLINE : OVERLAYS_NONE ) +
                ( ui->checkBox_showRowSums->isChecked() ? DIAG_ROWSUMS : OVERLAYS_NONE ) +
                ( ui->checkBox_showDerivOne->isChecked() ? DIAG_1ST_DERIV : OVERLAYS_NONE ) +
                ( ui->checkBox_showDerivTwo->isChecked() ? DIAG_2ND_DERIV : OVERLAYS_NONE ) +
                ( ui->checkBox_showRANSAC->isChecked() ? DIAG_RANSAC : OVERLAYS_NONE ) +
                ( ui->checkBox_showMoveROIs->isChecked() ? MOVE_ROIS : OVERLAYS_NONE ) +
                ( ui->checkBox_showMoveFind->isChecked() ? MOVE_FIND : OVERLAYS_NONE ) +
                ( ui->checkBox_showSearchROI->isChecked() ? SEARCH_ROI : OVERLAYS_NONE ) );
    gc::GC_STATUS retVal = m_visApp.GetImage( cv::Size( m_pQImg->width(), m_pQImg->height() ),
                                              static_cast< size_t >( m_pQImg->bytesPerLine() ),
                                              CV_8UC4, m_pQImg->scanLine( 0 ), nColorType, overlays );
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
    m_pLabelImgDisplay->setPixmap( pixmap );
}
void MainWindow::ScaleImage()
{
    if ( 0.0 > m_scaleFactor )
    {
        ui->statusBar->showMessage( "FAIL:  Invalid zoom factor" );
        return;
    }

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

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// event overloads
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::paintEvent( QPaintEvent * )
{
#ifdef _WIN32
    UpdatePixmap();
#endif
}
void MainWindow::on_tableAddRow( const string row_string ) { AddRow( row_string ); }
void MainWindow::on_updateProgress( const int value ) { emit sig_updateProgess( value ); }
void MainWindow::do_updateProgress( const int value )
{
    ui->progressBar_imageLoad->setValue( std::min( 100, std::max( 0, value ) ) );
}
void MainWindow::on_visAppMessage(string msg) { emit sig_visAppMessage( QString::fromStdString( msg ) ); }
void MainWindow::do_visAppMessage(QString msg)
{
    if ( msg.contains( "update image only" ) )
    {
        UpdatePixmap();
    }
    else
    {
        if ( msg.contains( "Folder run complete" ) )
        {
            ui->pushButton_findLine_processFolder->setEnabled( true );
            ui->pushButton_findLine_stopFolderProcess->setEnabled( false );
        }
        ui->textEdit_msgs->setText( msg );
    }
}
void MainWindow::mousePressEvent( QMouseEvent *pEvent )
{
    QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, pEvent->pos() );
    pt.setY( pt.y() - ui->mainToolBar->height() );
    int sensitivityRadius = qRound( 7.0 / m_scaleFactor );

    if ( nullptr != m_pRubberBand && ui->actionSetROI->isChecked() )
    {
        int nX = pt.x();
        int nY = pt.y();
        m_ptCapture.setX( nX );
        m_ptCapture.setY( nY );
        if ( abs( nX - m_rectRubberBand.left() ) < sensitivityRadius )
        {
            if ( abs( nY - m_rectRubberBand.top() ) < sensitivityRadius )               m_nCapturePos = 1;
            else if ( abs( nY - m_rectRubberBand.bottom() ) < sensitivityRadius )       m_nCapturePos = 32;
            else if ( nY > m_rectRubberBand.top() && nY < m_rectRubberBand.bottom() )   m_nCapturePos = 8;
            else m_nCapturePos = 0;
        }
        else if ( abs( nX - m_rectRubberBand.right() ) < sensitivityRadius )
        {
            if ( abs( nY - m_rectRubberBand.top() ) < sensitivityRadius )               m_nCapturePos = 4;
            else if ( abs( nY - m_rectRubberBand.bottom() ) < sensitivityRadius )       m_nCapturePos = 128;
            else if ( nY > m_rectRubberBand.top() && nY < m_rectRubberBand.bottom() )   m_nCapturePos = 16;
            else m_nCapturePos = 0;
        }
        else if ( nX > m_rectRubberBand.left() && nX < m_rectRubberBand.right() )
        {
            if ( abs ( nY - m_rectRubberBand.top() ) < sensitivityRadius )              m_nCapturePos = 2;
            else if ( abs ( nY - m_rectRubberBand.bottom() ) < sensitivityRadius )      m_nCapturePos = 64;
            else if ( nY > m_rectRubberBand.top() && nY < m_rectRubberBand.bottom() )   m_nCapturePos = 255;
            else m_nCapturePos = 0;
        }
        m_bCaptured = 0 < m_nCapturePos ? true : false;
    }
    else if ( ui->actionSetRuler->isChecked() )
    {
        QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, pEvent->pos() );
        pt.setY( pt.y() - ui->mainToolBar->height() );
        m_nCapturePos = 0;
        int nX = qRound( static_cast< double >( pt.x() ) / m_scaleFactor + 0.5 );
        int nY = qRound( static_cast< double >( pt.y() ) / m_scaleFactor + 0.5 );
        m_ptCapture.setX( nX );
        m_ptCapture.setY( nY );
        if ( abs( nX - m_lineOne.p1().x() ) < sensitivityRadius &&
             abs( nY - m_lineOne.p1().y() ) < sensitivityRadius ) m_nCapturePos = 1;
        else if ( abs( nX - m_lineOne.p2().x() ) < sensitivityRadius &&
                  abs( nY - m_lineOne.p2().y() ) < sensitivityRadius ) m_nCapturePos = 2;
        else
        {
            double dDist = DistToLine( static_cast< double >( nX ), static_cast< double >( nY ),
                                       static_cast< double >( m_lineOne.p1().x() ),
                                       static_cast< double >( m_lineOne.p1().y() ),
                                       static_cast< double >( m_lineOne.p2().x() ),
                                       static_cast< double >( m_lineOne.p2().y() ) );
            if ( 10 > qRound( dDist + 0.5 ) ) m_nCapturePos = 5;
        }
        m_bCaptured = 0 < m_nCapturePos ? true : false;
    }
}
void MainWindow::mouseMoveEvent( QMouseEvent *pEvent )
{
    QPoint pt = m_pLabelImgDisplay->mapFrom( ui->centralWidget, pEvent->pos() );
    pt.setY( pt.y() - ui->mainToolBar->height() );
    int nX = qRound( ( static_cast< double >( pt.x() ) / m_scaleFactor ) + 0.5 );
    int nY = qRound( ( static_cast< double >( pt.y() ) / m_scaleFactor ) + 0.5 );
    if ( 0 <= nX && m_imgWidth > nX && 0 <= nY && m_imgHeight > nY )
    {
        if ( !ui->actionSetRuler->isChecked() )
        {
            Point2d world;
            GC_STATUS retVal = m_visApp.PixelToWorld( Point2d( nX, nY ), world );
            if ( GC_OK != retVal )
            {
                world = Point2d( -9999999.9, -9999999.9 );
            }

            ui->textEdit_measures->setText( "PIXEL" );
            QString strMsg = QString( "X=" ) + QString::number( nX ) + " Y=" + QString::number( nY );
            ui->textEdit_measures->append( strMsg );
            ui->textEdit_measures->append( "WORLD" );
            strMsg = QString( "X1=" ) + QString::number( world.x ) + " Y1=" + QString::number( world.y );
            ui->textEdit_measures->append( strMsg );
        }
    }
    else
        ui->textEdit_msgs->setText( "Off image" );

    if ( m_bCaptured )
    {
        if ( nullptr != m_pRubberBand && ui->actionSetROI->isChecked() )
        {
            TestAgainstRubberBands( pt );
        }
        else if ( ui->actionSetRuler->isChecked() )
        {
            TestAgainstFindLines( pt );
            int nXpix1 = qRound( ( static_cast< double >( m_lineOne.p1().x() ) / m_scaleFactor ) + 0.5 );
            int nYpix1 = qRound( ( static_cast< double >( m_lineOne.p1().y() ) / m_scaleFactor ) + 0.5 );
            int nXpix2 = qRound( ( static_cast< double >( m_lineOne.p2().x() ) / m_scaleFactor ) + 0.5 );
            int nYpix2 = qRound( ( static_cast< double >( m_lineOne.p2().y() ) / m_scaleFactor ) + 0.5 );
            double lenPix = Distance( nXpix1, nYpix1, nXpix2, nYpix2 );

            Point2d world1, world2;
            GC_STATUS retVal1 = m_visApp.PixelToWorld( Point2d( nXpix1, nYpix1 ), world1 );
            if ( GC_OK != retVal1 )
            {
                world1 = Point2d( -9999999.9, -9999999.9 );
            }
            GC_STATUS retVal2 = m_visApp.PixelToWorld( Point2d( nXpix2, nYpix2 ), world2 );
            if ( GC_OK != retVal2 )
            {
                world2 = Point2d( -9999999.9, -9999999.9 );
            }
            double lenWorld = ( GC_OK != retVal1 || GC_OK != retVal2 ) ? -9999999.9 :  Distance( world1.x, world1.y, world2.x, world2.y );

            ui->textEdit_measures->setText( "PIXEL" );
            QString strMsg = QString( "X1=" ) + QString::number( nXpix1 ) + " Y1=" + QString::number( nYpix1 );
            strMsg += QString( " X2=" ) + QString::number( nXpix2 ) + " Y2=" + QString::number( nYpix2 );
            ui->textEdit_measures->append( strMsg );
            strMsg = QString( "Length=" ) + QString::number( lenPix );
            ui->textEdit_measures->append( strMsg );
            ui->textEdit_measures->append( "WORLD" );
            strMsg = QString( "X1=" ) + QString::number( world1.x ) + " Y1=" + QString::number( world1.y );
            ui->textEdit_measures->append( strMsg );
            strMsg = QString( "X2=" ) + QString::number( world2.x ) + " Y2=" + QString::number( world2.y );
            ui->textEdit_measures->append( strMsg );
            strMsg = QString( "Length=" ) + QString::number( lenWorld );
            ui->textEdit_measures->append( strMsg );
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
        ui->actionSetROI->setChecked( !ui->actionSetROI->isChecked() );
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
            filesystem::path p( m_imageFilePaths[ i ] );
            if ( filesystem::exists( p ) )
            {
                ui->listWidget_imageFolder->addItem( QString::fromStdString( p.filename().string() ) );
            }
        }
        if ( 0 < ui->listWidget_imageFolder->count() )
            ui->listWidget_imageFolder->setCurrentRow( 0 );
        on_actionZoomToFit_triggered();
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
void MainWindow::on_actionZoomToFit_triggered()
{
    ZoomTo( m_imgWidth, m_imgHeight );
}
void MainWindow::on_actionZoom100_triggered()
{
    ui->horizontalSlider_zoom->setValue( 100 );
}
void MainWindow::on_actionSetROI_toggled( bool )
{
    if ( ui->actionSetRuler->isChecked() )
        ui->actionSetRuler->setChecked( false );
    ScaleImage();
}
void MainWindow::on_actionSetRuler_triggered()
{
    if ( ui->actionSetROI->isChecked() )
    {
        ui->actionSetROI->setChecked( false );
    }
    UpdatePixmapTarget();
}
void MainWindow::on_actionImageLoad_triggered()
{
    QString filters = "Image Files (*.png *.jpg *.bmp)";
    QString strFullPath = QFileDialog::getOpenFileName( this,
        "Open Image", m_folderLoadImages, "Image Files (*.png *.jpg *.bmp)", &filters );
    if ( strFullPath.isEmpty() )
        return;

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
        return;
    }
    else
    {
        on_actionZoom100_triggered();
    }

    cv::Size sizeImg;
    ret = m_visApp.GetImageSize( sizeImg );
    if ( 0 != ret )
    {
        ui->statusBar->showMessage( "Get image size failed (on image load)" );
        return;
    }
    if ( sizeImg.width != m_imgWidth || sizeImg.height != m_imgWidth )
    {
        ret = ResizeImage( sizeImg.width, sizeImg.height );
        if ( 0 != ret )
            return;
    }
    ui->statusBar->showMessage( "Loaded: " + strFullPath );
}
void MainWindow::on_actionImageSave_triggered()
{
    QString filters = "Image Files (*.png *.jpg *.bmp)";
    QString strFullPath = QFileDialog::getSaveFileName( this,
        "Save Image", m_folderSaveImages, "Image Files (*.png *.jpg *.bmp)", &filters );

    if ( strFullPath.isEmpty() )
        return;

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
        return;
    }
    GC_STATUS retVal = m_visApp.SaveImage( strFullPath.toStdString(), static_cast< IMG_BUFFERS >( nColorType ) );
    if ( GC_OK != retVal )
        ui->statusBar->showMessage( "Save image failed" );
    else
        ui->statusBar->showMessage( "Saved: " + strFullPath );
}
void MainWindow::on_actionSaveVideo_triggered()
{
    QString filters = "Video Files (*.avi)";
    QString strFullPath = QFileDialog::getSaveFileName( this,
        "Save Video", m_folderSaveImages, "Video Files (*.avi)", &filters );

    if ( strFullPath.isEmpty() )
        return;

    QDir dirInfo( strFullPath );
    m_folderSaveImages = dirInfo.absolutePath();
    ui->statusBar->showMessage( "Video save not yet enabled" );

    // ui->statusBar->showMessage( 0 == ret ? "Save video: SUCCESS" : "Save video: FAIL" );
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
        return;
    }


    QDir dirInfo( strFullPath );
    m_folderLoadImages = dirInfo.absolutePath();
    ui->lineEdit_imageFolder->setText( strFullPath );
    on_lineEdit_imageFolder_textEdited( strFullPath );
}
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// vision calibration
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MainWindow::on_pushButton_visionCalibrate_clicked()
{
    QString strFolder = ui->lineEdit_imageFolder->text();
    if ( !strFolder.endsWith( '/') )
        strFolder += '/';

    QString strFilepath = strFolder + ui->listWidget_imageFolder->currentItem()->text();
    ui->textEdit_msgs->setText( "calibrating..." );
    ui->textEdit_msgs->update();
    ui->statusBar->showMessage( "calibrating..." );
    ui->statusBar->update();
    GC_STATUS retVal = m_visApp.Calibrate( strFilepath.toStdString(),
                                           ui->lineEdit_calibVisionTarget_csv->text().toStdString(),
                                           ui->lineEdit_calibVisionResult_json->text().toStdString() );
    ui->checkBox_showCalib->setChecked( true );
    m_pComboBoxImageToView->setCurrentText( "Overlay" );
    UpdatePixmapTarget();

    ui->statusBar->showMessage( QString( "Calibration: " ) + ( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
}
void MainWindow::on_toolButton_calibVisionTarget_csv_browse_clicked()
{
    QString strFullPath = QFileDialog::getOpenFileName( this, "Select calibration world coordinate CSV file", ui->lineEdit_calibVisionTarget_csv->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "No calibration world coordinate CSV file selected" );
    }
    else if (! strFullPath.contains( ".csv" ) )
    {
        ui->statusBar->showMessage( "File must have \".csv\" extension" );
    }
    else
    {
        ui->lineEdit_calibVisionTarget_csv->setText( strFullPath );
    }
}
void MainWindow::on_toolButton_calibVisionResult_json_browse_clicked()
{
    QString strFullPath = QFileDialog::getSaveFileName( this, "Set calibration json filepath", ui->lineEdit_calibVisionResult_json->text() );
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
void MainWindow::on_toolButton_findLine_annotatedResultFolder_browse_clicked()
{
    QString strFullPath = QFileDialog::getExistingDirectory( this, "Set annotated result image folder", ui->lineEdit_findLine_annotatedResultFolder->text() );
    if ( strFullPath.isNull() )
    {
        ui->statusBar->showMessage( "Could not get annotated result image folder path" );
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
        params.calibFilepath = ui->lineEdit_calibVisionResult_json->text().toStdString();
        params.timeStampFormat = ui->lineEdit_timestampFormat->text().toStdString();
        params.timeStampType = ui->radioButton_dateTimeInFilename->isChecked() ? FROM_FILENAME : FROM_EXIF;
        params.timeStampStartPos = ui->spinBox_timeStringPosZero->value();

        FindLineResult result;
        retVal = m_visApp.CalcLine( params, result );
        if ( GC_OK == retVal )
        {
            ui->checkBox_showFindLine->setChecked( true );
            m_pComboBoxImageToView->setCurrentText( "Overlay" );
            UpdatePixmapTarget();
            ui->statusBar->showMessage( "Find line: SUCCESS" );
        }
        else
        {
            ui->statusBar->showMessage( "Find line: FAILURE" );
        }
        ui->textEdit_msgs->clear();
        for ( size_t i = 0; i < result.msgs.size(); ++i )
        {
            ui->textEdit_msgs->append( QString::fromStdString( result.msgs[ i ] ) );
        }
    }
}
void MainWindow::on_pushButton_findLine_processFolder_clicked()
{
    FindLineParams params;
    string folder = ui->lineEdit_findLineTopFolder->text().toStdString();
    if ( '/' != folder[ folder.size() - 1 ] )
        folder += '/';
    params.calibFilepath = ui->lineEdit_calibVisionResult_json->text().toStdString();
    params.timeStampFormat = ui->lineEdit_timestampFormat->text().toStdString();
    params.timeStampType = ui->radioButton_dateTimeInFilename->isChecked() ? FROM_FILENAME : FROM_EXIF;
    params.timeStampStartPos = ui->spinBox_timeStringPosZero->value();
    params.resultImagePath = ui->checkBox_createFindLine_annotatedResults->isChecked() ? ui->lineEdit_findLine_annotatedResultFolder->text().toStdString() : "";
    params.resultCSVPath = ui->checkBox_createFindLine_csvResultsFile->isChecked() ? ui->lineEdit_findLine_resultCSVFile->text().toStdString() : "";

    ui->pushButton_findLine_processFolder->setEnabled( false );
    ui->pushButton_findLine_stopFolderProcess->setEnabled( true );

    vector< string > headings = { "filename", "timestamp", "water level" };
    InitTable( headings );
    GC_STATUS retVal = m_visApp.CalcLinesInFolder( folder, params, ui->radioButton_folderOfImages->isChecked() );

    ui->textEdit_msgs->clear();
    ui->textEdit_msgs->append( GC_OK == retVal ? "Folder run started" : "Folder run failed to start" );

    on_actionZoomToFit_triggered();
    m_pComboBoxImageToView->setCurrentText( "Overlay" );
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
        if ( strFullPath.endsWith( ".jpg" ) || strFullPath.endsWith( ".png" ) )
        {
            std::string data;
            GC_STATUS retVal = m_visApp.GetMetadata( strFullPath.toStdString(), data );
            ui->textEdit_msgs->setText( data.c_str() );
            ui->textEdit_msgs->append( "All data retrieved: " + QString( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
        }
    }
}
void MainWindow::ClearTable() { ui->tableWidget->clear(); }
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
        std::string data;
        GC_STATUS retVal = m_visApp.CreateAnimation( ui->lineEdit_imageFolder->text().toStdString(),
                                                     strFullPath.toStdString(), ui->doubleSpinBox_animateFPS->value(),
                                                     ui->doubleSpinBox_animateScale->value() );
        ui->textEdit_msgs->append( "Animation creation: " + QString( GC_OK == retVal ? "SUCCESS" : "FAILURE" ) );
    }
    else
    {
        ui->textEdit_msgs->append( "Animation creation: Invalid extension. Must be .gif" );
    }
}
void MainWindow::on_pushButton_test_clicked()
{
}

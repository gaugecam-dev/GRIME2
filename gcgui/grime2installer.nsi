;NSIS Modern User Interface
;Basic Example Script
;Written by Joost Verburg

!ifndef INSTALLFILEPATH_QT
!define INSTALLFILEPATH_QT "..\thirdparty\qt\qt_5_15_1\x64\bin"
!endif

!ifndef INSTALLFILEPATH_OPENCV
!define INSTALLFILEPATH_OPENCV "..\thirdparty\opencv\opencv_451\bin\vc19"
!endif

!ifndef INSTALLFILEPATH_BOOST
!define INSTALLFILEPATH_BOOST "..\thirdparty\boost\boost_1_74_0\vc141\bin"
!endif

!ifndef INSTALLFILEPATH_EXIFTOOL
!define INSTALLFILEPATH_EXIFTOOL "..\thirdparty\exiftool\exiftool_12_18"
!endif

!ifndef INSTALLFILEPATH_FFMPEG
!define INSTALLFILEPATH_FFMPEG "..\thirdparty\ffmpeg\bin"
!endif

!ifndef INSTALLFILEPATH_MSVC_REDIST
!define INSTALLFILEPATH_MSVC_REDIST "..\thirdparty\msvc_redist"
!endif

!ifndef INSTALLFILEPATH_RELEASE
!define INSTALLFILEPATH_RELEASE "..\build-gcgui-Desktop_Qt_5_15_1_MSVC2019_64bit-Release\release"
!endif

!ifndef INSTALLFILEPATH_DOCS
!define INSTALLFILEPATH_DOCS ".\docs"
!endif

!ifndef INSTALLFILEPATH_CONFIG
!define INSTALLFILEPATH_CONFIG ".\config"
!endif

!ifndef WIN_CONFIG_PATH
!define WIN_CONFIG_PATH "c:\gaugecam\config"
!endif

;--------------------------------
;Include Modern UI and logic lib

  !include "MUI.nsh"
  !include "LogicLib.nsh"

;--------------------------------
;General

Unicode True

  ;Name and file
  Name "GaugeCam"
  OutFile "GaugeCamInstaller.exe"
  RequestExecutionLevel admin ;Require admin rights on NT6+ (When UAC is turned on)

  ;Default installation folder
  InstallDir "$PROGRAMFILES\GaugeCam\grime2"

  ;Get installation folder from registry if available
  InstallDirRegKey HKCU "Software\GaugeCam\grime2" ""

;--------------------------------
;Interface Settings

  !define MUI_ABORTWARNING

;--------------------------------
;Pages

  !insertmacro MUI_PAGE_LICENSE ".\LICENSE"
  !insertmacro MUI_PAGE_COMPONENTS
  !insertmacro MUI_PAGE_DIRECTORY
  !insertmacro MUI_PAGE_INSTFILES

  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_INSTFILES

;--------------------------------
;Languages

    !insertmacro MUI_LANGUAGE "English"

;--------------------------------
;Installer Sections

Section "GaugeCam Files" grime2

  SetOutPath "$INSTDIR\docs"
  File "${INSTALLFILEPATH_DOCS}\perl_artistic_license.txt"
  File "${INSTALLFILEPATH_DOCS}\Background_installation_guideline.pdf"
  File "${INSTALLFILEPATH_DOCS}\Bowtie_Fiducial_Pattern.pdf"
  File "${INSTALLFILEPATH_DOCS}\Bowtie_Fiducial_Pattern.dwg"
  File "${INSTALLFILEPATH_DOCS}\boost_license.txt"
  File "${INSTALLFILEPATH_DOCS}\lgpl_license.txt"
  File "${INSTALLFILEPATH_DOCS}\release_notes.html"

  SetOutPath "${WIN_CONFIG_PATH}"
  File "${INSTALLFILEPATH_CONFIG}\calib_result.png"
  File "${INSTALLFILEPATH_CONFIG}\calibration_target_world_coordinates.csv"
  File "${INSTALLFILEPATH_CONFIG}\calib.json"
  File "${INSTALLFILEPATH_CONFIG}\settingsWin.cfg"

  SetOutPath "${WIN_CONFIG_PATH}\2012_demo\05"
  File "${INSTALLFILEPATH_CONFIG}\2012_demo\05\*.*"

  SetOutPath "${WIN_CONFIG_PATH}\2012_demo\06"
  File "${INSTALLFILEPATH_CONFIG}\2012_demo\06\*.*"

  SetOutPath "$INSTDIR\platforms"
  File "${INSTALLFILEPATH_QT}\plugins\platforms\qwindows.dll"

  SetOutPath "$INSTDIR\msvc_redist"
  File "${INSTALLFILEPATH_MSVC_REDIST}\VC_redist.x64.exe"

  SetOutPath "$INSTDIR"
  File "${INSTALLFILEPATH_RELEASE}\grime2.exe"
  File "${INSTALLFILEPATH_QT}\Qt5Core.dll"
  File "${INSTALLFILEPATH_QT}\Qt5Gui.dll"
  File "${INSTALLFILEPATH_QT}\Qt5Widgets.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_core451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_dnn451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_imgproc451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_imgcodecs451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_videoio451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_video451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_calib3d451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_flann451.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_features2d451.dll"
  File "${INSTALLFILEPATH_BOOST}\boost_system-vc141-mt-x64-1_74.dll"
  File "${INSTALLFILEPATH_BOOST}\boost_chrono-vc141-mt-x64-1_74.dll"
  File "${INSTALLFILEPATH_BOOST}\boost_date_time-vc141-mt-x64-1_74.dll"
  File "${INSTALLFILEPATH_BOOST}\boost_filesystem-vc141-mt-x64-1_74.dll"
  File "${INSTALLFILEPATH_EXIFTOOL}\ExifTool_install_12.18_64.exe"
  File "${INSTALLFILEPATH_FFMPEG}\ffmpeg.exe"

  ; add shortcuts
  CreateDirectory "$SMPROGRAMS\GaugeCam"
  CreateShortCut "$SMPROGRAMS\GaugeCam\grime2.lnk" "$INSTDIR\grime2.exe"
  CreateShortCut "$SMPROGRAMS\GaugeCam\Uninstall grime2.lnk" "$INSTDIR\Uninstall.exe"

  ;Store installation folder
  WriteRegStr HKCU "Software\GaugeCam\grime2" "" $INSTDIR

  ;Create uninstaller
  WriteUninstaller "$INSTDIR\Uninstall.exe"

SectionEnd

;--------------------------------
; Redistributable section
;--------------------------------
Section "CheckVCRedist"
  ReadRegStr $0 HKLM "SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.0\VC\Runtimes\X64" "Version"
  DetailPrint "Found version $0"
  ; Check for v14.27.29016.00 [sic]
  ${If} $0 >= "v14.27.29016.00"
     DetailPrint "The installed version is usable"
  ${Else}
     SetOutPath "$INSTDIR"
     ; File "${MSVS_DIR}\VC_redist.x64.exe"
     ExecWait '"$INSTDIR\msvc_redist\VC_redist.x64.exe"  /passive /norestart'
  ${EndIf}
  Delete "$INSTDIR\msvc_redist\*.*"
  RMDIR "$INSTDIR\temp"
SectionEnd

;--------------------------------
; Additional Prerequisites
;--------------------------------
Section -Prerequisites_exiftool
  IfFileExists "C:\Program Files\ExifTool\ExifTool.exe" endExifTool beginExifTool
    Goto endExifTool
    beginExifTool:
    MessageBox MB_OK "Your system does not appear to have ExifTool installed.$\n$\nPress OK to install it."
    File "${INSTALLFILEPATH_EXIFTOOL}\ExifTool_install_12.18_64.exe"
    ExecWait "${INSTALLFILEPATH_EXIFTOOL}\ExifTool_install_12.18_64.exe"
  endExifTool:
SectionEnd

;--------------------------------
; Uninstaller section
;--------------------------------
Section "Uninstall"

  Delete "$INSTDIR\*.*"
  Delete "$INSTDIR\docs\*.*"
  Delete "$INSTDIR\config\*.*"
  Delete "$INSTDIR\config\*.*"
  Delete "$INSTDIR\config\2012_demo\05\*.*"
  Delete "$INSTDIR\config\2012_demo\06\*.*"
  Delete "$INSTDIR\platforms\*.*"
  Delete "$INSTDIR\msvc_redist\*.*"
  Delete "${WIN_CONFIG_PATH}\*.*"
  Delete "${WIN_CONFIG_PATH}\2012_demo\05\*.*"
  Delete "${WIN_CONFIG_PATH}\2012_demo\06\*.*"

  RMDIR "$INSTDIR\docs"
  RMDIR "$INSTDIR\platforms"
  RMDIR "$INSTDIR\msvc_redist"
  RMDIR "$INSTDIR\"
  RMDIR "${WIN_CONFIG_PATH}"
  RMDIR "${WIN_CONFIG_PATH}\2012_demo\05"
  RMDIR "${WIN_CONFIG_PATH}\2012_demo\06"

  Delete "$SMPROGRAMS\GaugeCam\grime2\Uninstall.lnk"
  Delete "$SMPROGRAMS\GaugeCam\grime2\grime2.lnk"
  RMDIR "$SMPROGRAMS\GaugeCam\grime2"

SectionEnd

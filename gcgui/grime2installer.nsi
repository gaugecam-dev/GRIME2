;NSIS Modern User Interface
;Basic Example Script
;Written by Joost Verburg

!ifndef INSTALLFILEPATH_QT
!define INSTALLFILEPATH_QT "..\thirdparty\qt\qt_6.7.2\x64\vc19\bin"
!endif

!ifndef INSTALLFILEPATH_OPENCV
!define INSTALLFILEPATH_OPENCV "..\thirdparty\opencv\opencv_4.10.0\x64\vc19\bin"
!endif

; !ifndef INSTALLFILEPATH_BOOST
; !define INSTALLFILEPATH_BOOST "..\thirdparty\boost\boost_1.86\x64\vc19\bin"
; !endif

; !ifndef INSTALLFILEPATH_FFMPEG
; !define INSTALLFILEPATH_FFMPEG "..\thirdparty\ffmpeg\ffmpeg_432"
; !endif

!ifndef INSTALLFILEPATH_PREREQS
!define INSTALLFILEPATH_PREREQS "..\thirdparty\prereqs"
!endif

!ifndef INSTALLFILEPATH_RELEASE
!define INSTALLFILEPATH_RELEASE "..\build\Desktop_x86_windows_msvc2022_pe_64bit-Release"
!endif

!ifndef INSTALLFILEPATH_GCGUI
!define INSTALLFILEPATH_GCGUI "${INSTALLFILEPATH_RELEASE}\gcgui\release"
!endif

!ifndef INSTALLFILEPATH_GRIME2CLI
!define INSTALLFILEPATH_GRIME2CLI "${INSTALLFILEPATH_RELEASE}\grime2cli\release"
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

!ifndef WIN_RESULTS_PATH
!define WIN_RESULTS_PATH "c:\gaugecam\results"
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
  File "${INSTALLFILEPATH_DOCS}\boost_license.txt"
  File "${INSTALLFILEPATH_DOCS}\lgpl_license.txt"
  File "${INSTALLFILEPATH_DOCS}\release_notes.html"

  CreateDirectory "${WIN_RESULTS_PATH}"

  SetOutPath "${WIN_CONFIG_PATH}"
  File "${INSTALLFILEPATH_CONFIG}\calib_result.png"
  File "${INSTALLFILEPATH_CONFIG}\calib.json"
  File "${INSTALLFILEPATH_CONFIG}\settingsWin.cfg"

  SetOutPath "${WIN_CONFIG_PATH}\2022_demo"
  File "${INSTALLFILEPATH_CONFIG}\2022_demo\*.*"

  SetOutPath "$INSTDIR\platforms"
  File "${INSTALLFILEPATH_QT}\plugins\platforms\qwindows.dll"

  SetOutPath "$INSTDIR\prereqs"
  File "${INSTALLFILEPATH_PREREQS}\VC_redist.x64.exe"
  File "${INSTALLFILEPATH_PREREQS}\ExifTool_install_12.92_64.exe"

  SetOutPath "$INSTDIR"
  File "${INSTALLFILEPATH_GCGUI}\grime2.exe"
  File "${INSTALLFILEPATH_GRIME2CLI}\grime2cli.exe"
  File "${INSTALLFILEPATH_CONFIG}\batch_test_win.bat"
  File "${INSTALLFILEPATH_QT}\Qt6Core.dll"
  File "${INSTALLFILEPATH_QT}\Qt6Gui.dll"
  File "${INSTALLFILEPATH_QT}\Qt6Widgets.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_core4100.dll"
  ; File "${INSTALLFILEPATH_OPENCV}\opencv_dnn4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_imgproc4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_imgcodecs4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_videoio4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_video4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_calib3d4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_flann4100.dll"
  File "${INSTALLFILEPATH_OPENCV}\opencv_features2d4100.dll"
  ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ; now linked statically
  ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ; File "${INSTALLFILEPATH_BOOST}\boost_system-vc141-mt-x64-1_78.dll"
  ; File "${INSTALLFILEPATH_BOOST}\boost_chrono-vc141-mt-x64-1_78.dll"
  ; File "${INSTALLFILEPATH_BOOST}\boost_date_time-vc141-mt-x64-1_78.dll"
  ; File "${INSTALLFILEPATH_FFMPEG}\ffmpeg.exe"

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

  SetOutPath "$INSTDIR\prereqs"

  ReadRegStr $0 HKLM "SOFTWARE\WOW6432Node\Microsoft\VisualStudio\14.0\VC\Runtimes\X64" "Version"
  DetailPrint "Found version $0"
  ; Check for v14.27.29016.00 [sic]
  ${If} $0 >= "v14.27.29016.00"
     DetailPrint "The installed version is usable"
  ${Else}
     ; File "${MSVS_DIR}\VC_redist.x64.exe"
     ExecWait '"..\prereqs\VC_redist.x64.exe"  /passive /norestart'
  ${EndIf}
SectionEnd

;--------------------------------
; Additional Prerequisites
;--------------------------------
Section "Prerequisites_exiftool"
  SetOutPath "$INSTDIR\prereqs"

  IfFileExists "C:\Program Files\ExifTool\ExifTool.exe" alreadyInstalledExifTool beginExifTool
  beginExifTool:
    MessageBox MB_OK "Your system does not appear to have ExifTool installed.$\n$\nPress OK to install it."
    ExecWait "..\prereqs\ExifTool_install_12.92_64.exe"
    Goto endExifTool
  alreadyInstalledExifTool:
    DetailPrint "ExifTool already installed"
    Goto endExifTool
  endExifTool:
SectionEnd

;--------------------------------
; Clean up Prerequisites
;--------------------------------
Section "Cleanup_prerequisites"
  Delete "$INSTDIR\prereqs\*.*"
  RMDIR "$INSTDIR\prereqs"
SectionEnd

;--------------------------------
; Uninstaller section
;--------------------------------
Section "Uninstall"

  Delete "$INSTDIR\*.*"
  Delete "$INSTDIR\docs\*.*"
  Delete "$INSTDIR\config\*.*"
  Delete "$INSTDIR\config\*.*"
  Delete "$INSTDIR\config\\05\*.*"
  Delete "$INSTDIR\config\2022_demo\*.*"
  Delete "$INSTDIR\platforms\*.*"
  Delete "$INSTDIR\installers\*.*"
  Delete "${WIN_CONFIG_PATH}\*.*"
  Delete "${WIN_CONFIG_PATH}\2022_demo\*.*"

  RMDIR "$INSTDIR\docs"
  RMDIR "$INSTDIR\platforms"
  RMDIR "$INSTDIR\installers"
  RMDIR "$INSTDIR"
  RMDIR "${WIN_CONFIG_PATH}"
  RMDIR "${WIN_CONFIG_PATH}\2022_demo"

  Delete "$SMPROGRAMS\GaugeCam\grime2\Uninstall.lnk"
  Delete "$SMPROGRAMS\GaugeCam\grime2\grime2.lnk"
  RMDIR "$SMPROGRAMS\GaugeCam\grime2"

SectionEnd

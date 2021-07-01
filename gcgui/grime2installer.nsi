;NSIS Modern User Interface
;Basic Example Script
;Written by Joost Verburg

!ifndef INSTALLFILEPATH_QT
!define INSTALLFILEPATH_QT "..\thirdparty\qt\qt_5_15_1\x64\bin"
!endif

!ifndef INSTALLFILEPATH_OPENCV
!define INSTALLFILEPATH_OPENCV "..\thirdparty\opencv\opencv_451\bin\vc19"
!endif

; !ifndef INSTALLFILEPATH_BOOST
; !define INSTALLFILEPATH_BOOST "..\thirdparty\boost\boost_1_74_0\vc141\bin"
; !endif

; !ifndef INSTALLFILEPATH_FFMPEG
; !define INSTALLFILEPATH_FFMPEG "..\thirdparty\ffmpeg\ffmpeg_432"
; !endif

!ifndef INSTALLFILEPATH_PREREQS
!define INSTALLFILEPATH_PREREQS "..\thirdparty\prereqs"
!endif

!ifndef INSTALLFILEPATH_RELEASE
!define INSTALLFILEPATH_RELEASE "..\..\build-grime2-Desktop_Qt_5_15_1_MSVC2019_64bit-Release"
!endif

!ifndef INSTALLFILEPATH_GCGUI
!define INSTALLFILEPATH_GCGUI "${INSTALLFILEPATH_RELEASE}\gcgui\release"
!endif

!ifndef INSTALLFILEPATH_GRIM2CLI
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

  SetOutPath "$INSTDIR\prereqs"
  File "${INSTALLFILEPATH_PREREQS}\VC_redist.x64.exe"
  File "${INSTALLFILEPATH_PREREQS}\ExifTool_install_12.26_64.exe"

  SetOutPath "$INSTDIR"
  File "${INSTALLFILEPATH_GCGUI}\grime2.exe"
  File "${INSTALLFILEPATH_GRIME2CLI}\grime2cli.exe"
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
  ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ; now linked statically
  ; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ; File "${INSTALLFILEPATH_BOOST}\boost_system-vc141-mt-x64-1_74.dll"
  ; File "${INSTALLFILEPATH_BOOST}\boost_chrono-vc141-mt-x64-1_74.dll"
  ; File "${INSTALLFILEPATH_BOOST}\boost_date_time-vc141-mt-x64-1_74.dll"
  ; File "${INSTALLFILEPATH_BOOST}\boost_filesystem-vc141-mt-x64-1_74.dll"
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
var earliest
Section "Prerequisites_exiftool"
  SetOutPath "$INSTDIR\prereqs"

  IfFileExists "C:\Program Files\ExifTool\ExifTool.exe" testTexifTool beginExifTool
  testTexifTool:
    DetailPrint "Hello world!"
    nsExec::ExecToStack '"C:\Program Files\ExifTool\ExifTool.exe" -ver'
    Pop $0
    Pop $1
    ; ${VersionConvert} "0.15c+" "abcdefghijklmnopqrstuvwxyz+" $R0
    ; $R0="0.15.0327"

    ; ${VersionConvert} "0.15c" "abcdefghijklmnopqrstuvwxyz+" $R1
    ; $R1="0.15.03"
    ; $R0 = $1
    ; $R1="12.25"
    DetailPrint "$1"
    DetailPrint "12.25"

    ${VersionCompare} $1 "12.25" $R2
    ${If} $R2 < 2
      MessageBox MB_OK "Your system does not appear to have ExifTool installed.$\n$\nPress OK to install it."
      ExecWait "..\prereqs\ExifTool_install_12.26_64.exe"
    ${Else}
      DetailPrint "ExifTool already installed"
      Goto endExifTool
    ${EndIf}
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
  Delete "$INSTDIR\config\2012_demo\05\*.*"
  Delete "$INSTDIR\config\2012_demo\06\*.*"
  Delete "$INSTDIR\platforms\*.*"
  Delete "$INSTDIR\installers\*.*"
  Delete "${WIN_CONFIG_PATH}\*.*"
  Delete "${WIN_CONFIG_PATH}\2012_demo\05\*.*"
  Delete "${WIN_CONFIG_PATH}\2012_demo\06\*.*"

  RMDIR "$INSTDIR\docs"
  RMDIR "$INSTDIR\platforms"
  RMDIR "$INSTDIR\installers"
  RMDIR "$INSTDIR"
  RMDIR "${WIN_CONFIG_PATH}"
  RMDIR "${WIN_CONFIG_PATH}\2012_demo\05"
  RMDIR "${WIN_CONFIG_PATH}\2012_demo\06"

  Delete "$SMPROGRAMS\GaugeCam\grime2\Uninstall.lnk"
  Delete "$SMPROGRAMS\GaugeCam\grime2\grime2.lnk"
  RMDIR "$SMPROGRAMS\GaugeCam\grime2"

SectionEnd

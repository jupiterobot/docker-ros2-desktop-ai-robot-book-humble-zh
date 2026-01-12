#!/bin/bash
echo "entrypoint.sh version 20250120 (fixed)"

#set -eu
#set -v

for v in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY
do
  eval "test -v $v -a -z $"$v" && unset $v"
done

update-locale LANG=zh_CN.UTF-8

export LANG=zh_CN.UTF-8
export LANGUAGE=zh_CN:zh:Hans
export TZ=Asia/Shanghai

# VNC password
VNC_PASSWORD=${PASSWORD:-ubuntu}

# Setup VNC password file
mkdir -p "$HOME/.vnc"
echo "$VNC_PASSWORD" | vncpasswd -f > "$HOME/.vnc/passwd"
chmod 600 "$HOME/.vnc/passwd"
chown -R "$USER:$USER" "$HOME/.vnc"

# Patch noVNC UI to auto-fill password (for convenience)
if sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD';/" /usr/lib/novnc/app/ui.js; then
    echo "noVNC password patched."
else
    echo "Warning: Failed to patch noVNC ui.js"
fi

# Clear sensitive variables early
unset PASSWORD VNC_PASSWORD

# xstartup script for VNC session
XSTARTUP_PATH="$HOME/.vnc/xstartup"
cat << EOF > "$XSTARTUP_PATH"
#!/bin/sh
pulseaudio -D --enable-memfd=True
unset DBUS_SESSION_BUS_ADDRESS
export GTK_IM_MODULE=fcitx
export QT_IM_MODULE=fcitx
export XMODIFIERS=@im=fcitx
export DefaultIMModule=fcitx
fcitx
exec mate-session
EOF
chown "$USER:$USER" "$XSTARTUP_PATH"
chmod 755 "$XSTARTUP_PATH"

# vncserver launch script
rm -f /tmp/.X0-lock
if [ -z "$RESOLUTION" ]; then
    RESOLUTION=1920x1080
fi
VNCRUN_PATH="$HOME/.vnc/vnc_run.sh"
cat << EOF > "$VNCRUN_PATH"
#!/bin/sh
vncserver :0 -fg -geometry $RESOLUTION -depth 32 -localhost no
EOF
chown "$USER:$USER" "$VNCRUN_PATH"
chmod 755 "$VNCRUN_PATH"

# Supervisor config
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf
cat << EOF > "$CONF_PATH"
[supervisord]
nodaemon=true
user=root

[program:vnc]
command=gosu '$USER' bash '$VNCRUN_PATH'

[program:novnc]
command=gosu '$USER' bash -c "websockify --web=/usr/lib/novnc 80 localhost:5900"
EOF

# Add ～/bin and ～/.local/bin to PATH in .bashrc (if not already present)
PATCH='
if [[ -d "$HOME/bin" && ! "$PATH" =～ "$HOME/bin" ]]; then
    PATH="$HOME/bin:$PATH"
fi

if [[ -d "$HOME/.local/bin" && ! "$PATH" =～ "$HOME/.local/bin" ]]; then
    PATH="$HOME/.local/bin:$PATH"
fi
'

if ! grep -qF "$PATCH" "$HOME/.bashrc"; then
    echo "$PATCH" >> "$HOME/.bashrc"
fi

# Desktop shortcuts
mkdir -p "$HOME/Desktop"

# Terminator
cat << EOF > "$HOME/Desktop/terminator.desktop"
[Desktop Entry]
Name=Terminator
Comment=Multiple terminals in one window
TryExec=terminator
Exec=terminator
Icon=terminator
Type=Application
Categories=GNOME;GTK;Utility;TerminalEmulator;System;
StartupNotify=true
X-Ubuntu-Gettext-Domain=terminator
X-Ayatana-Desktop-Shortcuts=NewWindow;
Keywords=terminal;shell;prompt;command;commandline;
[NewWindow Shortcut Group]
Name=Open a New Window
Exec=terminator
TargetEnvironment=Unity
EOF
chown "$USER:$USER" "$HOME/Desktop/terminator.desktop"

# Firefox
cat << EOF > "$HOME/Desktop/firefox.desktop"
[Desktop Entry]
Version=1.0
Name=Firefox Web Browser
Name[zh_CN]=Firefox 网络浏览器
Comment=Browse the World Wide Web
Comment[zh_CN]=浏览互联网
GenericName=Web Browser
GenericName[zh_CN]=网络浏览器
Keywords=Internet;WWW;Browser;Web;Explorer;网页;浏览;上网;火狐;Firefox;ff;互联网;网站;
Exec=firefox %u
Terminal=false
Type=Application
Icon=firefox
Categories=GNOME;GTK;Network;WebBrowser;
MimeType=text/html;text/xml;application/xhtml+xml;application/xml;application/rss+xml;application/rdf+xml;image/gif;image/jpeg;image/png;x-scheme-handler/http;x-scheme-handler/https;x-scheme-handler/ftp;x-scheme-handler/chrome;video/webm;application/x-xpinstall;
StartupNotify=true
Actions=new-window;new-private-window;

[Desktop Action new-window]
Name=新建窗口
Name[en_US]=Open a New Window
Exec=firefox -new-window

[Desktop Action new-private-window]
Name=新建隐私窗口
Name[en_US]=Open a New Private Window
Exec=firefox -private-window
EOF
chown "$USER:$USER" "$HOME/Desktop/firefox.desktop"

# VSCodium icon
CODIUM_ICON_SRC="/usr/share/codium/resources/app/resources/linux/code.png"
if [ -f "$CODIUM_ICON_SRC" ]; then
    mkdir -p /usr/share/icons/hicolor/256x256/apps
    cp "$CODIUM_ICON_SRC" /usr/share/icons/hicolor/256x256/apps/codium.png
    gtk-update-icon-cache /usr/share/icons/hicolor || true
fi

# VSCodium desktop shortcut
cat << EOF > "$HOME/Desktop/codium.desktop"
[Desktop Entry]
Name=VSCodium
Comment=Code Editing. Redefined. (Open Source)
GenericName=Text Editor
Exec=env DONT_PROMPT_WSL_INSTALL=1 /usr/bin/codium --no-sandbox %F
Icon=codium
Type=Application
StartupNotify=false
StartupWMClass=VSCodium
Categories=TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;application/x-code-workspace;
Actions=new-empty-window;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/usr/bin/codium --new-window %F
Icon=codium
EOF
chown "$USER:$USER" "$HOME/Desktop/codium.desktop"
chmod +x "$HOME/Desktop/codium.desktop"

# Gazebo libEGL warning workaround (relax GPU device permissions)
# Note: This is broad; consider using video group in production
chmod a+rw /dev/dri/* 2>/dev/null || true

# === Install VSCodium extensions on first run ===
EXTENSIONS_FILE="$HOME/.vscodium-extensions-installed"
if [ ! -f "$EXTENSIONS_FILE" ]; then
    echo "Installing VSCodium extensions..."
    # Run as target user with DISPLAY set (even if X not fully up, CLI often works)
    su - "$USER" -c "DISPLAY=:0 codium --install-extension ms-python.python --force" || echo "Failed to install Python extension"
    su - "$USER" -c "DISPLAY=:0 codium --install-extension ms-ceintl.vscode-language-pack-zh-hans --force" || echo "Failed to install Chinese language pack"
    touch "$EXTENSIONS_FILE"
    chown "$USER:$USER" "$EXTENSIONS_FILE"
fi

# Start supervisord as PID 1
exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf
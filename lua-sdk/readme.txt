In this folder is documentation for Assetto Corsa CSP Lua API: definition files you can plug into
your text editor and see available functions, parameters they take, type of returned value, all
sorts of descriptions. Makes writing scripts a much easier and safer experience.

-----------------------------------
|  How to set it up in few steps  |
-----------------------------------

1. Install VS Code: https://code.visualstudio.com/

2. Open VS Code and install Lua extension by sumneko: go to File/Preferences/Extensions and 
  search for Lua, or go to https://marketplace.visualstudio.com/items?itemName=sumneko.lua and
  click “Install”.

3. Go to File/Preferences/Settings, search for “lua library”, find “Lua Workspace” option and 
  copy full path to one of folders inside “assettocorsa/extension/internal/lua-sdk” corresponding
  to the type of script you want to create.

  Alternatively, if you would want to create scripts of different types, you can install a custom
  extension “vs-code-extension.vsix” you can find next to this file. Simply go to 
  File/Preferences/Extensions, click on three dots, choose “Install from VSIX…” and select that
  file. This extension would scan folders in your current workspace opened in VS Code, detect
  the type of script you are working on, find AC root folder and automatically update library 
  path.

  For it to work though, you need to have a folder in a workspace which would contain folder with
  the script you are working on. For example, if you are working on car display scripts, it needs
  to be either in “…/content/cars/…” or in “…/extension/config/cars/…”. Check “rules.json” if you
  want to see other paths.

4. That’s all! Whole thing would activate automatically if you are editing Lua files, and help you
  by hinting about available functions and such.

  Please note: if you are working on car or track script, it wouldn’t work with INI files, but you
  can move script outside by replacing “SCRIPT='… your code …'” by “SCRIPT=some_file.lua” and 
  moving actual code to a file named “some_file.lua” next to the config.

5. If your Lua extension will complain about library files being too large, look for 
  “Lua.workspace.preloadFileSize” setting and increase its value to something like 1000 or 2000.

------------------
|  More details  |
------------------

If you are using some IDE other than VS Code, definitions are prepared in EmmyLua format, so you
can go and search for a plugin for your IDE: https://github.com/EmmyLua.

There are also README files generated for each type of library, but at the moment they’re far 
from ideal.

------------------
|  About LuaJIT  |
------------------

LuaJIT interpreter in this project is OpenResty fork of LuaJIT with few additional tweaks on top.
There are few more important details you should be aware of:

1. Main functions, such as `script.update()`, are bound on the first call, so CSP wouldn’t have
  to go looking for “script” → “update” for each call, so changing `script.update` live wouldn’t
  have an effect. If you want for something like this to work, use something like `script.myupdate`,
  and have `script.update` call `script.myupdate` instead.

2. Backported syntax extensions (https://luajit.org/extensions.html#lj30_bp_syntax) are available
  since 0.3.0-preview699. Make sure to update the VS Code plugin for the syntax highlighting and such.
  There is one important caveat, though, regarding arrow functions. In original LuaJIT, using anonymous
  functions (for example, for `table.map()`) means a new function object is created every time
  `table.map()` is called, which is where LuaJIT compiler stops, and code becomes slow. With CSP
  variant of LuaJIT, arrow functions that don’t capture local variables, are cached instead, greatly
  improving performance. However, it means that their identity remains the same. Basically, these two
  pieces of code would behave differently:

    local d = {}
    for _ = 1, 1000 do d[function () end] = true end
    print(table.nkeys(d)) --- 1000: each step, a new function is created, so we have 1000 unique functions

    local d = {}
    for _ = 1, 1000 do d[|| -> nil] = true end
    print(table.nkeys(d)) --- 1: a single captureless function is reused

3. Doing `local mmin = math.min` in the beginning of each Lua file for an extra bit of performance is
  no longer necessary.

4. There is a built-in preprocessor going over all the Lua code (except the code loaded with `loadstring`
  and such). For more information, check out this post:
  <https://github.com/ac-custom-shaders-patch/acc-lua-sdk/wiki/Lua-preprocessor>

5. Any CSP API can be one of two types, direct FFI calls, and regular Lua→C calls. Lua→C calls add about
  20 ns of overhead, so for many small often used functions, such as the ones from `ui` namespace, FFI calls
  are used. One issue with those is that if an exception is thrown within them, and the code around is
  JIT-compiled, Assetto Corsa will collapse. If used correctly, they won’t throw, but if, for example,
  you’re calling a `ui` method outside of UI-drawing context, it has nothing better to do but to throw,
  so please be careful. If CSP somehow manages to catch a throw like this, it’ll shut down the script for
  the rest of the session, to stay safe.

6. Generally, if Lua docs mismatch the behavior, follow the docs and report the issue to me to fix the behavior.

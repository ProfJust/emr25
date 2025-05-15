You are seeing this warning because `pip.exe` and related scripts were installed to a directory that is **not** included in your system's `PATH` environment variable. This means you won't be able to run `pip` (or any scripts installed by pip) from the command line unless you specify the full path, or you add that directory to your `PATH`[3][6].

To resolve this, you should add:

```
C:\Users\olafj\AppData\Local\Packages\PythonSoftwareFoundation.Python.3.13_qbz5n2kfra8p0\LocalCache\local-packages\Python313\Scripts
```

to your `PATH` environment variable.

## How to Add the Directory to PATH

1. **Open Environment Variables:**
   - Press `Windows + R`, type `SystemPropertiesAdvanced`, and press Enter.
   - Click the **Environment Variables…** button[1][6].

2. **Edit the Path Variable:**
   - Under **User variables** (recommended), select the `Path` variable and click **Edit**.
   - Click **New** and paste the path above[2][4][5].

3. **Apply and Close:**
   - Click **OK** on all dialogs to save your changes.

4. **Restart Your Terminal:**
   - Close and reopen your command prompt or PowerShell window to apply the changes[6].

## Verifying the Change

Open a new command prompt and run:

```powershell
pip --version
```

If everything is set up correctly, you should see the pip version information without needing to specify the full path.

---

**Summary:**  
Adding the Scripts directory to your PATH allows you to use pip and other installed scripts from any command prompt window, resolving the warning and making Python development smoother[3][6].

Citations:
[1] https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/
[2] https://realpython.com/add-python-to-path/
[3] https://github.com/python/cpython/issues/118498
[4] https://stackoverflow.com/questions/44272416/how-to-add-a-folder-to-path-environment-variable-in-windows-10-with-screensho
[5] https://superuser.com/questions/1861276/how-to-set-a-folder-to-the-path-environment-variable-in-windows-11
[6] https://www.eukhost.com/kb/how-to-add-to-the-path-on-windows-10-and-windows-11/
[7] https://www.youtube.com/watch?v=gb9e3m98avk
[8] https://stackoverflow.com/questions/39663091/how-can-i-add-a-python-script-to-the-windows-system-path
[9] https://stackoverflow.com/questions/75811420/add-python-entry-point-to-local-appdata
[10] https://www.java.com/en/download/help/path.html

---
Antwort von Perplexity: pplx.ai/share
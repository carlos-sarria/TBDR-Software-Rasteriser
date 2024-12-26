#include <Windows.h>
#include "draw_scene.h"

HBITMAP createBitmap(HWND window_handle)
{
    HBITMAP bitmap_handle;
    RECT client_rect;
    void *buffer;

    GetClientRect(window_handle, &client_rect);

    BITMAPINFO bitmap_info;
    memset(&bitmap_info, 0, sizeof(bitmap_info));
    bitmap_info.bmiHeader.biSize = sizeof(bitmap_info.bmiHeader);
    bitmap_info.bmiHeader.biWidth = client_rect.right - client_rect.left;
    bitmap_info.bmiHeader.biHeight = -(client_rect.bottom - client_rect.top);  // negative height for a top-down DIB
    bitmap_info.bmiHeader.biBitCount = 32;
    bitmap_info.bmiHeader.biPlanes = 1;
    bitmap_info.bmiHeader.biCompression = BI_RGB;

    bitmap_handle = CreateDIBSection(NULL, &bitmap_info, DIB_RGB_COLORS, &buffer, NULL, 0);

    return bitmap_handle;
}

void blit(HDC window_device_context, HBITMAP bitmap_handle)
{
    HDC memory_device_context = CreateCompatibleDC(window_device_context);
    HGDIOBJ replaced_gdi_object = SelectObject(memory_device_context, bitmap_handle);

    BITMAP bitmap;
    GetObject(bitmap_handle, sizeof(BITMAP), &bitmap);

    BitBlt(
        window_device_context,
        0,
        0,
        bitmap.bmWidth,
        bitmap.bmHeight,
        memory_device_context,
        0,
        0,
        SRCCOPY);

    SelectObject(memory_device_context, replaced_gdi_object);
    DeleteObject(memory_device_context);
}


LRESULT CALLBACK MessageCallback(HWND window_handle, UINT message, WPARAM wParam, LPARAM lParam)
{
	LRESULT result = 0;

    if (message == WM_DESTROY || (message ==WM_CHAR && wParam == VK_ESCAPE))
	{
		PostQuitMessage(0);
	}
	else
	{
		result = DefWindowProc(window_handle, message, wParam, lParam);
	}

	return result;
}

int CALLBACK WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	WNDCLASS window_class;
    memset(&window_class, 0, sizeof(window_class));
	window_class.lpfnWndProc = MessageCallback;
	window_class.hInstance = hInstance;
	window_class.hCursor = LoadCursor(hInstance, IDC_ARROW);
    window_class.lpszClassName = "RasterizerWindowClass";

	if (!RegisterClass(&window_class)) { return (int) GetLastError(); }

	HWND window_handle = CreateWindow(
		window_class.lpszClassName,
        "Software Rasterizer",
		WS_OVERLAPPEDWINDOW | WS_VISIBLE,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		NULL,
		NULL,
		hInstance,
		NULL);

    HBITMAP bitmap_handle = createBitmap(window_handle);

    BITMAP bitmap;
    GetObject(bitmap_handle, sizeof(BITMAP), &bitmap);

	MSG msg;
	BOOL running = TRUE;

    POINT mousePoint;
    unsigned char keyPressed;
    bool mousePressed = false;

    initialise_app("..\\model\\test.glb", bitmap.bmWidth, bitmap.bmHeight, bitmap.bmBits);

    // Initialise camera
    GetCursorPos(&mousePoint);
    updateCamera(0, false, mousePoint.x, mousePoint.y);

	while (running)
    {
        draw_frame();
        HDC window_device_context = GetDC(window_handle);
        blit(window_device_context, bitmap_handle);
        ReleaseDC(window_handle, window_device_context);

		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT) { running = FALSE; }

            mousePressed = GetAsyncKeyState(VK_LBUTTON);
            GetCursorPos(&mousePoint);

            for (keyPressed=0; keyPressed<128; keyPressed++) if (GetAsyncKeyState(keyPressed) & (1<<15)) break;
            updateCamera(keyPressed, mousePressed, mousePoint.x, mousePoint.y);

			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

    free_all();
	return msg.wParam;
}

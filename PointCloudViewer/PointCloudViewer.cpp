// PointCloudViewer.cpp : Defines the entry point for the application.
//

/*
* Example of a Windows OpenGL program.
* The OpenGL code is the same as that used in
* the X Window System sample
*/
#include <windows.h> 
#include <windowsx.h>
#include <shellapi.h>

#include <stdlib.h>

#include <GL/gl.h> 
#include <GL/glu.h> 

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <limits>
#include <cstddef>

#include <Eigen/Dense>

#include "point_cloud.h"

#define MAIN_LOGGER
#define LOGGING_LEVEL_1
#include "logger.hpp"

using namespace std;
using namespace Eigen;
using namespace pcv;

/* Windows globals, defines, and prototypes */
CHAR szAppName[] = "Win OpenGL";
HWND  ghWnd;
HDC   ghDC;
HGLRC ghRC;

#define SWAPBUFFERS SwapBuffers(ghDC) 
#define BLACK_INDEX     0 
#define RED_INDEX       13 
#define GREEN_INDEX     14 
#define BLUE_INDEX      16 
#define WIDTH           300 
#define HEIGHT          200 

LONG WINAPI MainWndProc(HWND, UINT, WPARAM, LPARAM);
BOOL bSetupPixelFormat(HDC);
void refresh_display_lists();

/* OpenGL globals, defines, and prototypes */
float translate_camera_speed;

float rotate_camera_speed;

float edge_length;

float gl_point_size;

int last_mouse_pos_x;
int last_mouse_pos_y;

struct c_viewport {
    c_viewport() {
    }
    c_viewport(int x, int y, int width, int height) {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }
    int x;
    int y;
    int width;
    int height;
};
c_viewport viewport;

#define POINT_CLOUD 1

GLvoid resize_viewport(GLsizei, GLsizei);
GLvoid initializeGL(GLsizei, GLsizei);
GLvoid drawScene(GLvoid);

string input_point_cloud_file_path;
string output_point_cloud_file_path;

c_point_cloud point_cloud;

enum class c_interactive_mode: int
{
    idle,
    selecting_points_to_hide,
    updating_points_to_hide,
    selecting_points_to_keep,
    updating_points_to_keep,
    filter_label
};
c_interactive_mode interactive_mode = c_interactive_mode::idle;

Vector3i filtered_label = Vector3i(1, 0, 0);

Vector2i min_visible;
Vector2i max_visible;

Vector3f eye_pnt;
Vector3f lookat_pnt;

bool get_command_line_options(vector< string >& arg_list) {
    int i;
    // requests parameters to be passed in pairs
    if (arg_list.size() % 2)
        return false;

    for (i = 0; i < arg_list.size(); i = i + 2) {
        if (arg_list[i] == "-i") {
            input_point_cloud_file_path = arg_list[i + 1];
        }
        if (arg_list[i] == "-o") {
            output_point_cloud_file_path = arg_list[i + 1];
        }
    }

    if (output_point_cloud_file_path.size() == 0) {
        output_point_cloud_file_path = input_point_cloud_file_path + ".out.kin";
    }

    return true;
}

void reset_camera_position() {
    // note, that in the opengl the camera looks at the negative z direction of the camera reference frame
    // position the camera looking at the object center

    lookat_pnt = (point_cloud.min_coord + point_cloud.max_coord) / 2.0f;
    eye_pnt = lookat_pnt - Vector3f(0, 0, point_cloud.max_coord(2) - point_cloud.min_coord(2));
}

void reset_settings() {
    filtered_label = Vector3i(1,0,0);

    translate_camera_speed = 0.03f;
    
    rotate_camera_speed = 0.01f;

    edge_length = 0.01f;

    gl_point_size = 3.0f;

    point_cloud.reset_visibility();

    reset_camera_position();

    interactive_mode = c_interactive_mode::idle;
}

bool get_point_screen_coordinate(const c_point_cloud_point& point, const Matrix4f& model_view_matrix, const Matrix4f& projection_matrix, Vector2i& point_screen_coordinates) {
    Vector4f point_coordinates;
    point_coordinates(3) = 1.0f;
    for (int i = 0; i < 3; i++) {
        point_coordinates(i) = point.X(i);
    }

    // see http://www.songho.ca/opengl/gl_transform.html#projection and http://webglfactory.blogspot.com/2011/05/how-to-convert-world-to-screen.html
    Vector4f clip_coordinates = projection_matrix*model_view_matrix*point_coordinates;
    Vector3f normalized_device_coordinates(clip_coordinates(0) / clip_coordinates(3), clip_coordinates(1) / clip_coordinates(3), clip_coordinates(2) / clip_coordinates(3));
    point_screen_coordinates(0) = (int)((float)viewport.width*(normalized_device_coordinates(0) + 1.0) / 2.0) + viewport.x;
    point_screen_coordinates(1) = (int)((float)viewport.height*(-normalized_device_coordinates(1) + 1.0) / 2.0) + viewport.y;
    return true;
}

void update_points_visibility() {
    Vector2i point_screen_coordinates;

    GLfloat model_view_matrix_raw[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat*)&model_view_matrix_raw);
    Matrix4f model_view_matrix;
    for (int i = 0; i < 4; i++) {
        model_view_matrix.col(i) << model_view_matrix_raw[i * 4], model_view_matrix_raw[i * 4 + 1], model_view_matrix_raw[i * 4 + 2], model_view_matrix_raw[i * 4 + 3];
    }

    GLfloat projection_matrix_raw[16];
    glGetFloatv(GL_PROJECTION_MATRIX, (GLfloat*)&projection_matrix_raw);
    Matrix4f projection_matrix;
    for (int i = 0; i < 4; i++) {
        projection_matrix.col(i) << projection_matrix_raw[i * 4], projection_matrix_raw[i * 4 + 1], projection_matrix_raw[i * 4 + 2], projection_matrix_raw[i * 4 + 3];
    }

    if (interactive_mode == c_interactive_mode::filter_label) {
        bool found_label = false;
        for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
            if (it->Label == filtered_label) {
                found_label = true;
                it->visible = 1;
            }
            else {
                it->visible = 0;
            }
        }
        if (found_label) {
            filtered_label += Vector3i(1, 0, 0);
        }
        else {
            for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
                it->visible = 1;
            }
            filtered_label = Vector3i(1, 0, 0);
        }
    }
    else {
        for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
            if (!it->visible)
                continue;

            if (get_point_screen_coordinate(*it, model_view_matrix, projection_matrix, point_screen_coordinates)) {
                bool point_in_selected_box = true;
                for (int i = 0; i < 2; i++) {
                    if (point_screen_coordinates(i) < min_visible(i) || point_screen_coordinates(i) > max_visible(i)) {
                        point_in_selected_box = false;
                        break;
                    }
                }

                if (interactive_mode == c_interactive_mode::updating_points_to_keep) {
                    if (!point_in_selected_box) {
                        it->visible = 0;
                    }
                }
                else
                    if (interactive_mode == c_interactive_mode::updating_points_to_hide) {
                        if (point_in_selected_box) {
                            it->visible = 0;
                        }
                    }
            }
        }
    }
    refresh_display_lists();
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    MSG        msg;
    WNDCLASS   wndclass;

    LPWSTR *szArglist;
    int nArgs;
    int i;

    vector< string > arg_list;

    reset_settings();

    szArglist = CommandLineToArgvW(GetCommandLineW(), &nArgs);
    if (NULL == szArglist) return 0;
    // skip the first argument, which is the full path of the executable
    for (i = 1; i < nArgs; i++) {
        size_t buf_len = wcslen(szArglist[i])+1;  // with the null terminator
        char* buf = (char*)calloc(buf_len+1, 1);
        if (!buf) return 0;
        size_t ret_len;
        errno_t ret = wcstombs_s(&ret_len, buf, buf_len, szArglist[i], buf_len);
        string s = buf;
        arg_list.push_back(s);
        free(buf);
    }
    // Free memory allocated for CommandLineToArgvW arguments.
    LocalFree(szArglist);

    if (!get_command_line_options(arg_list))
        return 0;

    /* Register the frame class */
    wndclass.style = 0;
    wndclass.lpfnWndProc = (WNDPROC)MainWndProc;
    wndclass.cbClsExtra = 0;
    wndclass.cbWndExtra = 0;
    wndclass.hInstance = hInstance;
    wndclass.hIcon = LoadIcon(hInstance, szAppName);
    wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
    wndclass.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wndclass.lpszMenuName = szAppName;
    wndclass.lpszClassName = szAppName;

    if (!RegisterClass(&wndclass))
        return FALSE;

    /* Create the frame */
    ghWnd = CreateWindow(szAppName,
        "Point Cloud Viewer",
        WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        WIDTH,
        HEIGHT,
        NULL,
        NULL,
        hInstance,
        NULL);

    /* make sure window was created */
    if (!ghWnd)
        return FALSE;

    /* show and update main window */
//    ShowWindow(ghWnd, nCmdShow);

    ShowWindow(ghWnd, SW_MAXIMIZE);
    
    UpdateWindow(ghWnd);

    /* animation loop */
    while (1) {
        /*
        *  Process all pending messages
        */

        while (PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE) == TRUE)
        {
            if (GetMessage(&msg, NULL, 0, 0))
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
            else {
                return TRUE;
            }
        }
        drawScene();
    }
}

/* main window procedure */
LONG WINAPI MainWndProc(
    HWND    hWnd,
    UINT    uMsg,
    WPARAM  wParam,
    LPARAM  lParam)
{
    LONG    lRet = 0;
    PAINTSTRUCT    ps;
    RECT rect;

    int xPos, yPos;
    int fwKeys, zDelta, mult;

    switch (uMsg) {

    case WM_CREATE:
        ghDC = GetDC(hWnd);
        if (!bSetupPixelFormat(ghDC))
            PostQuitMessage(0);

        ghRC = wglCreateContext(ghDC);
        wglMakeCurrent(ghDC, ghRC);
        GetClientRect(hWnd, &rect);
        initializeGL(rect.right, rect.bottom);
        break;

    case WM_PAINT:
        BeginPaint(hWnd, &ps);
        EndPaint(hWnd, &ps);
        return 0;

    case WM_SIZE:
        GetClientRect(hWnd, &rect);
        resize_viewport(rect.right, rect.bottom);
        break;

    case WM_CLOSE:
        if (ghRC)
            wglDeleteContext(ghRC);
        if (ghDC)
            ReleaseDC(hWnd, ghDC);
        ghRC = 0;
        ghDC = 0;

        DestroyWindow(hWnd);
        break;

    case WM_DESTROY:
        if (ghRC)
            wglDeleteContext(ghRC);
        if (ghDC)
            ReleaseDC(hWnd, ghDC);

        PostQuitMessage(0);
        break;

    case WM_KEYDOWN:
        switch (wParam)
        {
        case VK_LEFT:
            // Process the LEFT ARROW key. 
            break;
        case VK_RIGHT:
            // Process the RIGHT ARROW key. 
            break;
        case VK_UP:
            // Process the UP ARROW key. 
            break;
        case VK_DOWN:
            // Process the DOWN ARROW key. 
            break;
        case VK_HOME:
            // Process the HOME key. 
            break;
        case VK_END:
            // Process the END key. 
            break;
        case VK_INSERT:
            // Process the INS key. 
            break;
        case VK_DELETE:
            // Process the DEL key. 
            break;
        case VK_F2:
            // Process the F2 key. 
            break;
        default:
            // Process other non-character keystrokes. 
            break;
        }
        break;
    case WM_CHAR:
        switch (wParam)
        {
        case 0x08:
            // Process a backspace. 
            break;
        case 0x0A:
            // Process a linefeed. 
            break;
        case 0x1B:
            // Process an escape. 
            PostMessage(hWnd, WM_CLOSE, 0, 0);
            break;
        case 0x09:
            // Process a tab. 
            break;
        case 0x0D:
            // Process a carriage return. 
            break;

        case 0x44:
        case 0x64:
            // Process D, d
            translate_camera_speed /= 1.2f;
            rotate_camera_speed /= 1.2f;
            break;

        case 0x45:
        case 0x65:
            // Process E, e
            break;
        case 0x46:
        case 0x66:
            // Process F, f
            interactive_mode = c_interactive_mode::filter_label;
            break;
        case 0x48:
        case 0x68:
            // Process H, h
            interactive_mode = c_interactive_mode::selecting_points_to_hide;
            min_visible = Vector2i(INT_MAX, INT_MAX);
            max_visible = Vector2i(INT_MIN, INT_MIN);
            break;
        case 0x4B:
        case 0x6B:
            // Process K, k
            interactive_mode = c_interactive_mode::selecting_points_to_keep;
            min_visible = Vector2i(INT_MAX, INT_MAX);
            max_visible = Vector2i(INT_MIN, INT_MIN);
            break;

        case 0x50:
        case 0x70:
            // Process P, p
            gl_point_size += 1.0f;
            break;
        case 0x52:
        case 0x72:
            // Process R, r
            reset_settings();
            refresh_display_lists();
            break;

        case 0x53:
        case 0x73:
            // Process S, s
            if (output_point_cloud_file_path.size() > 0) {
                point_cloud.write_point_cloud_file(output_point_cloud_file_path);
            }
            break;

        case 0x55:
        case 0x75:
            // Process U, u
            translate_camera_speed *= 1.2f;
            rotate_camera_speed *= 1.2f;
            break;

        default:
            // Process displayable characters. 
//            ch = (TCHAR)wParam;
            break;
        }
        break;

    case WM_RBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;

        SetCapture(hWnd);

        return 0;

    case WM_RBUTTONUP:
        ReleaseCapture();
        break;

    case WM_LBUTTONDOWN:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;

        SetCapture(hWnd);

        break;

    case WM_LBUTTONUP:
        if (interactive_mode == c_interactive_mode::selecting_points_to_hide) {
            interactive_mode = c_interactive_mode::updating_points_to_hide;
        }
        if (interactive_mode == c_interactive_mode::selecting_points_to_keep) {
            interactive_mode = c_interactive_mode::updating_points_to_keep;
        }
        
        ReleaseCapture();
        break;

    case WM_MOUSEMOVE:
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        if (interactive_mode == c_interactive_mode::selecting_points_to_hide ||
            interactive_mode == c_interactive_mode::selecting_points_to_keep
            ) {
            if (fwKeys & MK_LBUTTON) {
                if (xPos < min_visible(0))
                    min_visible(0) = xPos;
                if (yPos < min_visible(1))
                    min_visible(1) = yPos;
                if (xPos > max_visible(0))
                    max_visible(0) = xPos;
                if (yPos > max_visible(1))
                    max_visible(1) = yPos;
            }
        } 
        else 
        if (interactive_mode == c_interactive_mode::idle) {
            if (fwKeys & MK_LBUTTON) {
                Vector3f translation = Vector3f(-(last_mouse_pos_y - yPos), last_mouse_pos_x - xPos,0).cross(lookat_pnt-eye_pnt);
                translation.normalize();
                translation = -translation*translate_camera_speed;
                eye_pnt = eye_pnt + translation;
                lookat_pnt = lookat_pnt + translation;
            }
            else 
            if (fwKeys & MK_RBUTTON) {
                Vector3f ang_vel = rotate_camera_speed*Vector3f((last_mouse_pos_y - yPos), -(last_mouse_pos_x - xPos), 0);
                // Rodrigues' formula
                Matrix3f ang_vel_hat;
                ang_vel_hat << 0.0, -ang_vel(2), ang_vel(1),
                    ang_vel(2), 0.0, -ang_vel(0),
                    -ang_vel(1), ang_vel(0), 0.0;
                float ang_vel_val = ang_vel.norm();

                if (ang_vel_val > 0) {
                    //Matrix3f rotation = Matrix3f::Identity() + ang_vel_hat*sin(ang_vel_val) / ang_vel_val + ang_vel_hat*ang_vel_hat / (ang_vel_val*ang_vel_val)*(1 - cos(ang_vel_val));
                    Matrix3f rotation = Matrix3f::Identity() + ang_vel_hat; // approximation for small ang_vel_val
                    eye_pnt = lookat_pnt + rotation*(eye_pnt - lookat_pnt);
                }
            }
        }

        last_mouse_pos_x = xPos;
        last_mouse_pos_y = yPos;
        break;
        
    case WM_MOUSEWHEEL:
        fwKeys = GET_KEYSTATE_WPARAM(wParam);
        zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
        xPos = GET_X_LPARAM(lParam);
        yPos = GET_Y_LPARAM(lParam);
        mult = zDelta / 120;
        if (interactive_mode == c_interactive_mode::idle) {
            Vector3f translation = mult*(lookat_pnt - eye_pnt);
            translation.normalize();
            translation = translation*translate_camera_speed;
            eye_pnt = eye_pnt + translation;
            lookat_pnt = lookat_pnt + translation;
        }
        break;

    default:
        lRet = (LONG)DefWindowProc(hWnd, uMsg, wParam, lParam);
        break;
    }

    return lRet;
}

#ifdef COLOR_INDEX
BOOL bSetupPixelFormat(HDC hdc)
{
    PIXELFORMATDESCRIPTOR pfd, *ppfd;
    int pixelformat;

    ppfd = &pfd;

    ppfd->nSize = sizeof(PIXELFORMATDESCRIPTOR);
    ppfd->nVersion = 1;
    ppfd->dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL |
        PFD_DOUBLEBUFFER;
    ppfd->dwLayerMask = PFD_MAIN_PLANE;
    ppfd->iPixelType = PFD_TYPE_COLORINDEX;
    ppfd->cColorBits = 8;
    ppfd->cDepthBits = 16;
    ppfd->cAccumBits = 0;
    ppfd->cStencilBits = 0;

    pixelformat = ChoosePixelFormat(hdc, ppfd);

    if ((pixelformat = ChoosePixelFormat(hdc, ppfd)) == 0)
    {
        MessageBox(NULL, "ChoosePixelFormat failed", "Error", MB_OK);
        return FALSE;
    }

    if (SetPixelFormat(hdc, pixelformat, ppfd) == FALSE)
    {
        MessageBox(NULL, "SetPixelFormat failed", "Error", MB_OK);
        return FALSE;
    }

    return TRUE;
}
#endif

BOOL bSetupPixelFormat(HDC hdc)
{
    PIXELFORMATDESCRIPTOR pfd = {
        sizeof(PIXELFORMATDESCRIPTOR),   // size of this pfd  
        1,                     // version number  
        PFD_DRAW_TO_WINDOW |   // support window  
        PFD_SUPPORT_OPENGL |   // support OpenGL  
        PFD_DOUBLEBUFFER,      // double buffered  
        PFD_TYPE_RGBA,         // RGBA type  
        24,                    // 24-bit color depth  
        0, 0, 0, 0, 0, 0,      // color bits ignored  
        0,                     // no alpha buffer  
        0,                     // shift bit ignored  
        0,                     // no accumulation buffer  
        0, 0, 0, 0,            // accum bits ignored  
        32,                    // 32-bit z-buffer  
        0,                     // no stencil buffer  
        0,                     // no auxiliary buffer  
        PFD_MAIN_PLANE,        // main layer  
        0,                     // reserved  
        0, 0, 0                // layer masks ignored  
    };

    int  iPixelFormat;

    // get the best available match of pixel format for the device context   
    iPixelFormat = ChoosePixelFormat(hdc, &pfd);

    // make that the pixel format of the device context  
    SetPixelFormat(hdc, iPixelFormat, &pfd);
    return TRUE;
}

/* OpenGL code */
GLvoid resize_viewport(GLsizei width, GLsizei height)
{
    GLfloat aspect;

    viewport = c_viewport(0, 0, width, height);
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);

    aspect = (GLfloat)width / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // the OpenGL projection transformation (not to be confused with the projective transformation in the computer vision) transforms 
    // all vertex data from the eye coordinates to the clip coordinates.

    float zNear = (point_cloud.max_coord(2) - point_cloud.min_coord(2)) / 100.0f;;
    float zFar = FLT_MAX;  // zNear + (point_cloud.max_coord(2) - point_cloud.min_coord(2))*10.0f;
    gluPerspective(45.0, aspect, zNear, zFar);

    glMatrixMode(GL_MODELVIEW);
}

void refresh_display_lists() {
    glNewList(POINT_CLOUD, GL_COMPILE);

    for (auto it = point_cloud.points.begin(); it != point_cloud.points.end(); ++it) {
        if (it->visible == 0)
            continue;
        glBegin(GL_POINTS);
        glColor3ub((GLubyte)it->Clr(0), (GLubyte)it->Clr(1), (GLubyte)it->Clr(2));
        glVertex3f(it->X(0), it->X(1), it->X(2));
        glEnd();

        if (it->Vector != Vector3f::Zero()) {
            glBegin(GL_LINES);
            glColor3ub(255, 0, 0);
            Vector3f normalized_edge = it->Vector;
            normalized_edge.normalize();
            Vector3f v = it->X + normalized_edge * edge_length;

            glVertex3f(it->X(0), it->X(1), it->X(2));
            glVertex3f(v(0), v(1), v(2));

            glEnd();
        }
    }

    glEndList();
}

GLvoid initializeGL(GLsizei width, GLsizei height)
{
    point_cloud.read_point_cloud_file(input_point_cloud_file_path);

    glClearIndex((GLfloat)BLACK_INDEX);
    glClearDepth(1.0);

    glEnable(GL_DEPTH_TEST);

    // white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    reset_camera_position();

#ifdef LOGGING
//    LOG("initializeGL: point_cloud.points.size = ", point_cloud.points.size(), " width = ", width, " height = ", height);
#endif
    resize_viewport(width, height);

    refresh_display_lists();
}

GLvoid drawScene(GLvoid)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPointSize(gl_point_size);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();

    gluLookAt(eye_pnt(0), eye_pnt(1), eye_pnt(2),
        lookat_pnt(0), lookat_pnt(1), lookat_pnt(2),
        0.0, 1.0, 0.0);

    if (interactive_mode == c_interactive_mode::updating_points_to_hide ||
        interactive_mode == c_interactive_mode::updating_points_to_keep ||
        interactive_mode == c_interactive_mode::filter_label
        ) {
        update_points_visibility();
        interactive_mode = c_interactive_mode::idle;
    }

    glCallList(POINT_CLOUD);
     
    SWAPBUFFERS;
}
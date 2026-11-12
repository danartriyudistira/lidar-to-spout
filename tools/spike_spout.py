"""Spike: validasi GL context + Spout texture/fbo sharing di mesin ini."""
import time
import glfw
import moderngl
import SpoutGL

W, H = 3840, 1080

if not glfw.init():
    raise SystemExit("glfw init failed")
glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
win = glfw.create_window(640, 180, "spike", None, None)
if not win:
    raise SystemExit("window failed")
glfw.make_context_current(win)

ctx = moderngl.create_context()
print("GL:", ctx.info["GL_RENDERER"], "|", ctx.info["GL_VERSION"])

tex = ctx.texture((W, H), 4)
fbo = ctx.framebuffer(color_attachments=[tex])

sender = SpoutGL.SpoutSender()
sender.setSenderName("Lidar Spike")
fbo.use()
ctx.clear(1.0, 0.4, 0.0, 1.0)
ok = sender.sendFbo(fbo.glo, W, H, False)
print("sendFbo first:", ok, "| GLDX ready:", sender.isGLDXReady())
print("sender info:", sender.getSenderInfo("Lidar Spike"))

t0 = time.perf_counter()
frames = 120
for i in range(frames):
    fbo.use()
    ctx.clear(0.1 + 0.5 * (i % 2), 0.3, 0.6, 1.0)
    sender.sendFbo(fbo.glo, W, H, False)
    glfw.poll_events()
el = (time.perf_counter() - t0) / frames
print(f"avg frame send: {el*1000:.2f} ms  -> {1/el:.0f} fps possible")

sender.releaseSender()
glfw.terminate()

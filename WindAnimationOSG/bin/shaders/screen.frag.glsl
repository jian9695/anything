uniform sampler2D u_screen;
uniform float u_opacity;

varying vec2 v_tex_pos;

void main() {
    vec2 v_tex_pos = gl_TexCoord[0].xy;
    vec4 color = texture2D(u_screen, v_tex_pos);
    // a hack to guarantee opacity fade out even with a value close to 1.0
    gl_FragColor = vec4(floor(255.0 * color * u_opacity) / 255.0);
}

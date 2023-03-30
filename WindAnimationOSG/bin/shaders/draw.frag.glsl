uniform sampler2D u_wind;
uniform sampler2D u_color_ramp;
uniform vec2 u_wind_min;
uniform vec2 u_wind_max;
uniform float u_opacity;
varying vec2 v_particle_pos;
varying vec2 v_tex_pos;

void main() {
    vec4 wind = texture2D(u_wind, v_particle_pos);
    gl_FragColor = wind;
}

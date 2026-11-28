"""Metallic-roughness shading for the photoreal mesh pass."""

PBR_FRAGMENT = """
#version 330 core
in vec3 world_position;
in vec3 world_normal;
uniform vec3 color;
uniform bool checker;
uniform vec3 camera_position;
uniform vec2 material;
uniform bool room_proxy;
out vec4 pixel;

const float pi = 3.14159265;

vec3 decode_srgb(vec3 rgb) {
    return mix(pow((rgb + 0.055) / 1.055, vec3(2.4)), rgb / 12.92,
               lessThanEqual(rgb, vec3(0.04045)));
}

vec3 encode_srgb(vec3 rgb) {
    return mix(1.055 * pow(rgb, vec3(1.0 / 2.4)) - 0.055, 12.92 * rgb,
               lessThanEqual(rgb, vec3(0.0031308)));
}

float smith(float cosine, float alpha_squared) {
    return 2.0 * cosine / max(cosine + sqrt(alpha_squared +
        (1.0 - alpha_squared) * cosine * cosine), 0.0001);
}

void main() {
    if (room_proxy) {
        pixel = vec4(vec3(0.24), 0.0);
        return;
    }

    vec3 surface = clamp(color, 0.0, 1.0);
    if (checker) {
        vec2 tile = floor(world_position.xy * 2.0);
        surface *= mix(1.0, 0.75, mod(tile.x + tile.y, 2.0));
    }
    vec3 base = decode_srgb(surface);
    float metallic = clamp(material.x, 0.0, 1.0);
    float roughness = clamp(material.y, 0.05, 1.0);
    float alpha_squared = pow(roughness, 4.0);
    vec3 view_direction = normalize(camera_position - world_position);
    vec3 normal = normalize(world_normal);
    if (dot(normal, view_direction) < 0.0) normal = -normal;
    vec3 light = normalize(vec3(-0.4, -0.6, 1.0));
    vec3 halfway = normalize(light + view_direction);
    float nl = max(dot(normal, light), 0.0);
    float nv = max(dot(normal, view_direction), 0.0);
    float nh = max(dot(normal, halfway), 0.0);
    float vh = clamp(dot(view_direction, halfway), 0.0, 1.0);

    vec3 f0 = mix(vec3(0.04), base, metallic);
    vec3 fresnel = f0 + (1.0 - f0) * pow(1.0 - vh, 5.0);
    float denominator = nh * nh * (alpha_squared - 1.0) + 1.0;
    float distribution = alpha_squared / (pi * denominator * denominator);
    float visibility = smith(nl, alpha_squared) * smith(nv, alpha_squared);
    vec3 specular = fresnel * distribution * visibility / max(4.0 * nl * nv, 0.0001);
    vec3 diffuse = (1.0 - fresnel) * (1.0 - metallic) * base / pi;
    vec3 direct = (diffuse + specular) * nl * vec3(2.5, 2.4, 2.2);
    vec3 hemisphere = mix(vec3(0.16, 0.13, 0.11), vec3(0.38, 0.43, 0.51),
                          normal.z * 0.5 + 0.5);
    vec3 linear_color = direct + hemisphere * (base * (1.0 - metallic) + f0 * 0.5);
    pixel = vec4(encode_srgb(linear_color / (1.0 + linear_color)), 1.0);
}
"""

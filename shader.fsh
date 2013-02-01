uniform sampler3D volume;
uniform sampler2D transfer;
varying vec3 texCoord;

float vol(vec3 coord) {
	vec2 sample = texture3D(volume, coord).xy;
	return (sample.x * (1.0 / 255.0) + sample.y * (254.0 / 255.0));
}

vec3 Gradient(vec3 coord) {
	const float offset = 0.002f;

	vec3 gradient = vec3( vol(vec3(coord.x + offset, coord.y, coord.z)) - 
						  vol(vec3(coord.x - offset, coord.y, coord.z)),
						  vol(vec3(coord.x, coord.y + offset, coord.z)) - 
						  vol(vec3(coord.x, coord.y - offset, coord.z)),
						  vol(vec3(coord.x, coord.y, coord.z + offset)) - 
						  vol(vec3(coord.x, coord.y, coord.z - offset))  );
	return normalize(gradient);
}

#define SHADING:

#define TRANSFER_FUNC


vec3 GetColor(float v) {
#ifdef TRANSFER_FUNC
	return texture2D(transfer, v);
#else
	return vec3(v, v, v);
#endif
}

#ifndef SHADING

void main()
{
	float v = vol(texCoord);
	vec3 col = GetColor(v);
	gl_FragColor = vec4(col.x, col.y, col.z, v * v * v * v * 0.5);
//	gl_FragColor = vec4(1,1,1,0.1);
}

#else

void main()
{
	float v = vol(texCoord);
	vec3 gradient = Gradient(texCoord);
//	float dist = distance(texCoord.xy, vec2(0.5, 0.5));

//	if(dist > 0.497)
//		v = 0.0;
//	v = v * 10.0;


	float alpha = 0.0f;
	if(v > 0.5)
		alpha = 1.0f;

	gl_FragColor.rgb = GetColor(v);
	gl_FragColor.a = alpha;
	gl_FragColor.rgb *= 0.25f + max(gradient.x, 0.0) * 0.75;
}

#endif

#version 120

uniform sampler2D Texture;
uniform int Width;
uniform float odw;

void main()
{
	vec3 Color = vec3(0.0);
	int wp1 = Width + 1;
	float Sum = 0.0;
	
	for(int x = -Width; x <= Width; x++)
	{
		float width = (wp1 - abs(float(x)));
		Color += texture2D(Texture, gl_TexCoord[0].st + vec2(odw * x, 0.0)).rgb * width;
		Sum += width;
	}
	
	gl_FragColor = vec4(Color / Sum, 1.0);
}

void old_main()
{	
	// offsets (distances from the center pixel to sample)
	float width = 1024.0;
	float offset[5] = float[](0.0, 0.1/width, 0.2/width, 0.3/width, 0.4/width);
	//float offset[3] = float[](0.0, 1.3846153846/width, 3.2307692308/width);
	
	// weights genreated from a guassian distribution
	float weight[5] = float[](0.2270270270, 0.1945945946, 0.12162116, 0.0540540541, 0.0162162162);
	//float weight[3] = float[](0.2270270270, 0.3162162162, 0.0702702703);
	
	float sum = 0.6135130519;
	
	// color of the center pixel
	vec3 FragmentColor = texture2D(Texture, gl_TexCoord[0].xy).rgb * weight[0]; 
	
	for (int i=1; i<5; i++){
	
		//sample texture color to the right
		FragmentColor += texture2D(Texture, gl_TexCoord[0].xy + vec2(offset[i], 0.0)).rgb * weight[i]; 
		
		// sample texture color to the left
		FragmentColor += texture2D(Texture, gl_TexCoord[0].xy - vec2(offset[i], 0.0)).rgb * weight[i]; 
	}
	
	
	gl_FragColor = vec4(FragmentColor, 1.0);
	
		
}
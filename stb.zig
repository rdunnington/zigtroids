pub usingnamespace @cImport({
    @cDefine("STB_IMAGE_IMPLEMENTATION", {});
    @cDefine("STBI_NO_SIMD", {});
    @cDefine("STBI_NO_HDR", {});
    @cDefine("STBI_FAILURE_USERMSG", {});
    // @cDefine("STBI_NO_TGA", {}); // stbi__tga_test uses goto
    @cInclude("stb_image.h");
});

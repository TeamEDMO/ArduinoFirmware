#include "Communications/DummyCommStream.h"

void DummyCommStream::init() {}
void DummyCommStream::write(const uint8_t *data, size_t length) {}
void DummyCommStream::write(const uint8_t byte) {}
void DummyCommStream::write(const char *const data, size_t length) {}
void DummyCommStream::update() {}
void DummyCommStream::begin() {}
void DummyCommStream::end() {}

DummyCommStream dummyComms{};
#ifndef CRYPTO_HMAC_H
#define CRYPTO_HMAC_H

#include <stdint.h>
#include <stddef.h>

/**
 * \brief HMAC template class.
 *
 * \tparam Hash The hash algorithm to use.
 */
template <class Hash>
class HMAC
{
public:
    /**
     * \brief Constructs a new HMAC context.
     *
     * \param key Points to the key.
     * \param len Length of the key in bytes.
     */
    HMAC(const uint8_t *key, size_t len)
    {
        memset(innerPad, 0x36, sizeof(innerPad));
        memset(outerPad, 0x5C, sizeof(outerPad));
        if (len > Hash::BLOCK_SIZE) {
            Hash hash;
            hash.reset();
            hash.update(key, len);
            hash.finalize(keyHash, sizeof(keyHash));
            key = keyHash;
            len = Hash::HASH_SIZE;
        }
        for (size_t i = 0; i < len; ++i) {
            innerPad[i] ^= key[i];
            outerPad[i] ^= key[i];
        }
        hash.reset();
        hash.update(innerPad, sizeof(innerPad));
    }

    /**
     * \brief Adds more data to the message.
     *
     * \param data Points to the data to add.
     * \param len Length of the data in bytes.
     */
    void update(const void *data, size_t len)
    {
        hash.update(data, len);
    }

    /**
     * \brief Finishes the HMAC and returns the digest.
     *
     * \param digest Points to the buffer to receive the digest.
     * \param len Length of the buffer in bytes.
     */
    void finalize(uint8_t *digest, size_t len)
    {
        uint8_t innerDigest[Hash::HASH_SIZE];
        hash.finalize(innerDigest, sizeof(innerDigest));
        hash.reset();
        hash.update(outerPad, sizeof(outerPad));
        hash.update(innerDigest, sizeof(innerDigest));
        hash.finalize(digest, len);
    }

private:
    uint8_t innerPad[Hash::BLOCK_SIZE];
    uint8_t outerPad[Hash::BLOCK_SIZE];
    uint8_t keyHash[Hash::HASH_SIZE];
    Hash hash;
};

#endif
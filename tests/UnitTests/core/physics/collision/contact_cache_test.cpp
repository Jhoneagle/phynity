#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/physics/collision/contact/contact_cache.hpp>
#include <core/physics/collision/contact/contact_manifold.hpp>

using namespace phynity::physics::collision;
using namespace phynity::math::vectors;

TEST_CASE("ContactManifold - Generate contact ID", "[collision][contact_cache]") {
    SECTION("Same ID for A-B and B-A") {
        uint64_t id_ab = ContactManifold::generate_contact_id(10, 20);
        uint64_t id_ba = ContactManifold::generate_contact_id(20, 10);
        
        REQUIRE(id_ab == id_ba);
    }
    
    SECTION("Different IDs for different pairs") {
        uint64_t id_10_20 = ContactManifold::generate_contact_id(10, 20);
        uint64_t id_10_30 = ContactManifold::generate_contact_id(10, 30);
        uint64_t id_20_30 = ContactManifold::generate_contact_id(20, 30);
        
        REQUIRE(id_10_20 != id_10_30);
        REQUIRE(id_10_20 != id_20_30);
        REQUIRE(id_10_30 != id_20_30);
    }
    
    SECTION("Update contact ID on manifold") {
        ContactManifold manifold;
        manifold.object_a_id = 5;
        manifold.object_b_id = 15;
        manifold.update_contact_id();
        
        uint64_t expected_id = ContactManifold::generate_contact_id(5, 15);
        REQUIRE(manifold.contact_id == expected_id);
    }
}

TEST_CASE("ContactCache - Basic operations", "[collision][contact_cache]") {
    ContactCache cache;
    
    SECTION("Empty cache") {
        REQUIRE(cache.empty());
        REQUIRE(cache.size() == 0);
    }
    
    SECTION("Add and retrieve contact") {
        ContactManifold manifold;
        manifold.object_a_id = 1;
        manifold.object_b_id = 2;
        manifold.contact.position = Vec3f(1.0f, 0.0f, 0.0f);
        manifold.contact.penetration = 0.5f;
        
        std::vector<ContactManifold> manifolds = {manifold};
        auto cached = cache.update(manifolds);
        
        REQUIRE_FALSE(cache.empty());
        REQUIRE(cache.size() == 1);
        REQUIRE(cached.size() == 1);
    }
    
    SECTION("Clear cache") {
        ContactManifold manifold;
        manifold.object_a_id = 1;
        manifold.object_b_id = 2;
        
        cache.update({manifold});
        REQUIRE(cache.size() == 1);
        
        cache.clear();
        REQUIRE(cache.empty());
    }
}

TEST_CASE("ContactCache - Contact persistence and age tracking", "[collision][contact_cache]") {
    ContactCache cache;
    
    ContactManifold manifold;
    manifold.object_a_id = 10;
    manifold.object_b_id = 20;
    manifold.contact.position = Vec3f(0.0f, 0.0f, 0.0f);
    manifold.contact.penetration = 0.3f;
    
    SECTION("New contact has age 0") {
        auto cached = cache.update({manifold});
        REQUIRE(cached.size() == 1);
        REQUIRE(cached[0].age == 0);
    }
    
    SECTION("Persistent contact ages correctly") {
        // Frame 1: First contact
        auto cached1 = cache.update({manifold});
        REQUIRE(cached1[0].age == 0);
        
        // Frame 2: Same contact persists
        auto cached2 = cache.update({manifold});
        REQUIRE(cached2[0].age == 1);
        
        // Frame 3: Still persisting
        auto cached3 = cache.update({manifold});
        REQUIRE(cached3[0].age == 2);
        
        // Frame 4
        auto cached4 = cache.update({manifold});
        REQUIRE(cached4[0].age == 3);
    }
    
    SECTION("Contact disappears and reappears resets age") {
        // Frame 1: Contact appears
        auto cached1 = cache.update({manifold});
        REQUIRE(cached1[0].age == 0);
        
        // Frame 2: Contact persists
        auto cached2 = cache.update({manifold});
        REQUIRE(cached2[0].age == 1);
        
        // Frame 3: Contact disappears (empty update)
        auto cached3 = cache.update({});
        REQUIRE(cached3.size() == 0);
        
        // Frame 4: Contact reappears (should be treated as new)
        auto cached4 = cache.update({manifold});
        REQUIRE(cached4[0].age == 0);
    }
}

TEST_CASE("ContactCache - Warm-start impulse caching", "[collision][contact_cache]") {
    ContactCache cache;
    
    ContactManifold manifold;
    manifold.object_a_id = 5;
    manifold.object_b_id = 15;
    manifold.contact.position = Vec3f(0.0f, 1.0f, 0.0f);
    manifold.update_contact_id();
    
    SECTION("New contact has zero previous impulse") {
        auto cached = cache.update({manifold});
        REQUIRE(cached[0].previous_impulse.x == 0.0f);
        REQUIRE(cached[0].previous_impulse.y == 0.0f);
        REQUIRE(cached[0].previous_impulse.z == 0.0f);
    }
    
    SECTION("Store and retrieve impulse") {
        // Frame 1: Create contact
        auto cached1 = cache.update({manifold});
        uint64_t contact_id = cached1[0].contact_id;
        
        // Simulate impulse being applied
        Vec3f applied_impulse(10.0f, -5.0f, 2.0f);
        cache.store_impulse(contact_id, applied_impulse);
        
        // Frame 2: Contact persists, should have warm-start data
        auto cached2 = cache.update({manifold});
        REQUIRE(cached2[0].previous_impulse.x == 10.0f);
        REQUIRE(cached2[0].previous_impulse.y == -5.0f);
        REQUIRE(cached2[0].previous_impulse.z == 2.0f);
    }
    
    SECTION("Multiple contacts with different impulses") {
        ContactManifold manifold_a = manifold;
        manifold_a.object_a_id = 1;
        manifold_a.object_b_id = 2;
        manifold_a.update_contact_id();
        
        ContactManifold manifold_b = manifold;
        manifold_b.object_a_id = 3;
        manifold_b.object_b_id = 4;
        manifold_b.update_contact_id();
        
        // Frame 1: Create both contacts
        auto cached1 = cache.update({manifold_a, manifold_b});
        cache.store_impulse(cached1[0].contact_id, Vec3f(1.0f, 0.0f, 0.0f));
        cache.store_impulse(cached1[1].contact_id, Vec3f(0.0f, 2.0f, 0.0f));
        
        // Frame 2: Both persist with different impulses
        auto cached2 = cache.update({manifold_a, manifold_b});
        
        // Find the manifolds (order may vary)
        ContactManifold* found_a = nullptr;
        ContactManifold* found_b = nullptr;
        
        for (auto& m : cached2) {
            if (m.contact_id == cached1[0].contact_id) found_a = &m;
            if (m.contact_id == cached1[1].contact_id) found_b = &m;
        }
        
        REQUIRE(found_a != nullptr);
        REQUIRE(found_b != nullptr);
        REQUIRE(found_a->previous_impulse.x == 1.0f);
        REQUIRE(found_b->previous_impulse.y == 2.0f);
    }
}

TEST_CASE("ContactCache - Multiple contacts and cleanup", "[collision][contact_cache]") {
    ContactCache cache;
    
    SECTION("Track multiple simultaneous contacts") {
        ContactManifold m1;
        m1.object_a_id = 1;
        m1.object_b_id = 2;
        
        ContactManifold m2;
        m2.object_a_id = 3;
        m2.object_b_id = 4;
        
        ContactManifold m3;
        m3.object_a_id = 5;
        m3.object_b_id = 6;
        
        auto cached = cache.update({m1, m2, m3});
        REQUIRE(cached.size() == 3);
        REQUIRE(cache.size() == 3);
    }
    
    SECTION("Remove disappeared contacts") {
        ContactManifold m1;
        m1.object_a_id = 1;
        m1.object_b_id = 2;
        
        ContactManifold m2;
        m2.object_a_id = 3;
        m2.object_b_id = 4;
        
        // Frame 1: Two contacts
        cache.update({m1, m2});
        REQUIRE(cache.size() == 2);
        
        // Frame 2: Only one contact remains
        cache.update({m1});
        REQUIRE(cache.size() == 1);
        
        // Frame 3: No contacts
        cache.update({});
        REQUIRE(cache.size() == 0);
    }
}

TEST_CASE("ContactCache - Determinism", "[collision][contact_cache]") {
    ContactCache cache1;
    ContactCache cache2;
    
    ContactManifold manifold;
    manifold.object_a_id = 7;
    manifold.object_b_id = 13;
    manifold.contact.position = Vec3f(1.0f, 2.0f, 3.0f);
    manifold.contact.penetration = 0.25f;
    
    SECTION("Same inputs produce same outputs") {
        auto cached1 = cache1.update({manifold});
        auto cached2 = cache2.update({manifold});
        
        REQUIRE(cached1[0].contact_id == cached2[0].contact_id);
        REQUIRE(cached1[0].age == cached2[0].age);
        REQUIRE(cached1[0].previous_impulse == cached2[0].previous_impulse);
    }
    
    SECTION("Repeated operations are deterministic") {
        for (int i = 0; i < 10; ++i) {
            auto c1 = cache1.update({manifold});
            auto c2 = cache2.update({manifold});
            
            REQUIRE(c1[0].age == c2[0].age);
            REQUIRE(c1[0].contact_id == c2[0].contact_id);
        }
    }
}

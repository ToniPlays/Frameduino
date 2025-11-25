#ifndef FRAMEDUINO_LINKED_LIST
#define FRAMEDUINO_LINKED_LIST

namespace Frameduino
{

    template <typename T>
    class linked_list
    {
    private:
        struct linked_list_node
        {
            T value;
            linked_list_node *next;
            linked_list_node(const T &val) : value(val), next(nullptr) {}
        };

        linked_list_node *head = nullptr;

    public:
        linked_list() = default;
        ~linked_list() { clear(); }

        // Disable copying for safety
        linked_list(const linked_list &) = delete;
        linked_list &operator=(const linked_list &) = delete;

        // Add a value to the head of the list
        void add(const T& value)
        {
            linked_list_node *node = new linked_list_node(value);
            node->next = head;
            head = node;
        }

        // Remove the first entry matching predicate
        bool remove(bool (*match)(T*, void*), void* usr_data)
        {
            linked_list_node *curr = head;
            linked_list_node *prev = nullptr;

            while (curr)
            {
                if (match(&curr->value, usr_data))
                {
                    if (prev)
                        prev->next = curr->next;
                    else
                        head = curr->next;

                    delete curr;
                    return true;
                }
                prev = curr;
                curr = curr->next;
            }
            return false;
        }

        // Find object matching predicate
        T *find(bool (*match)(T*, void*), void* user_data)
        {
            linked_list_node *curr = head;
            while (curr)
            {
                if (match(&curr->value, user_data))
                    return &curr->value;

                curr = curr->next;
            }
            return nullptr;
        }

        // Iterate list
        template <typename Func>
        void for_each(Func func)
        {
            linked_list_node *curr = head;
            while (curr)
            {
                func(curr->value);
                curr = curr->next;
            }
        }

        // Free all nodes
        void clear()
        {
            linked_list_node *curr = head;
            while (curr)
            {
                linked_list_node *next = curr->next;
                delete curr;
                curr = next;
            }
            head = nullptr;
        }
    };
}
#endif
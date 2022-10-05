package com.example.taxi.ui.home.provider

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.databinding.ItemUserListBinding
import com.gun0912.tedpermission.provider.TedPermissionProvider.context

class ProviderAdapter: RecyclerView.Adapter<ProviderAdapter.ProviderViewHolder>() {
    var taxiUserList = mutableListOf<TaxiUser>()
    lateinit var onTaxiUserClickListener: (View, Int) -> Unit
    lateinit var context: Context

    fun setListData(data: MutableList<TaxiUser>){
        taxiUserList = data
    }

    fun updateList(list: MutableList<TaxiUser>){
        this.taxiUserList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ProviderViewHolder {
        return ProviderViewHolder(
            ItemUserListBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onTaxiUserClickListener)
        }
    }

    override fun onBindViewHolder(holder: ProviderViewHolder, position: Int) {
        holder.bind(taxiUserList[position])
    }

    override fun getItemCount(): Int {
        return taxiUserList.size
    }

    class ProviderViewHolder(private val binding: ItemUserListBinding) :
        RecyclerView.ViewHolder(binding.root) {

        lateinit var place : String
        private lateinit var address : String
        private lateinit var latitude : String
        private lateinit var longitude : String

        fun bind(data: TaxiUser) {
            if(data.userImage != ""){
                Glide.with(context)
                    .load(data.userImage)
                    .into(binding.imageUserList)
            }else{
                Glide.with(context)
                    .load(R.drawable.img_user)
                    .into(binding.imageUserList)
            }
        }

        fun bindOnItemClickListener(onItemClickListener: (View, Int) -> Unit ) {
            binding.root.setOnClickListener {
                onItemClickListener(it, bindingAdapterPosition)
            }
        }
    }

}